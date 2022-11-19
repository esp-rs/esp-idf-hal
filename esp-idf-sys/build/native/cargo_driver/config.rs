use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::{anyhow, bail, Context, Result};
use cargo_metadata::{Metadata, Package};
use embuild::cargo::IntoWarning;
use embuild::espidf::parse_esp_idf_git_ref;
use embuild::utils::PathExt;
use embuild::{cmake, git};
use serde::Deserialize;

use crate::config::utils::{parse_from_env, set_when_none};
use crate::config::EspIdfSys;

pub const ESP_IDF_VERSION_VAR: &str = "ESP_IDF_VERSION";
pub const ESP_IDF_REPOSITORY_VAR: &str = "ESP_IDF_REPOSITORY";

pub const DEFAULT_ESP_IDF_VERSION: &str = "v4.4.3";
pub const DEFAULT_CMAKE_GENERATOR: cmake::Generator = {
    // No Ninja builds for linux=aarch64 from Espressif yet
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    {
        cmake::Generator::UnixMakefiles
    }

    #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
    {
        cmake::Generator::Ninja
    }
};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct NativeConfig {
    /// The version of the esp-idf to use.
    #[serde(deserialize_with = "parse::git_ref")]
    pub esp_idf_version: Option<git::Ref>,
    /// The URL to the git repository of the `esp-idf`.
    pub esp_idf_repository: Option<String>,
    /// The cmake generator to use when cmake builds the esp-idf.
    #[serde(deserialize_with = "parse::cmake_generator")]
    esp_idf_cmake_generator: Option<cmake::Generator>,

    /// The path to the esp-idf repository.
    pub idf_path: Option<PathBuf>,

    /// Additional components to build and maybe generate bindings for.
    ///
    /// Can be specified in the root crate's `package.metadata.esp-idf-sys` and all direct
    /// dependencies'.
    ///
    /// This option is not available as an environment variable.
    #[serde(alias = "extra-components")]
    pub extra_components: Vec<ExtraComponent>,

    /// A list of esp-idf components (names) that should be built. This list is used to
    /// trim the esp-idf build. Any component that is a dependency of a component in this
    /// list will also automatically be built.
    ///
    /// If this option is not specified, all components will be built. Note though that
    /// some components must be explicitly enabled in the sdkconfig.
    #[serde(default, deserialize_with = "parse::list")]
    pub esp_idf_components: Option<Vec<String>>,
}

impl NativeConfig {
    pub fn try_from_env() -> Result<NativeConfig> {
        Ok(parse_from_env(&["extra_components"])?)
    }

    /// Get the user-specified esp-idf version or [`DEFAULT_ESP_IDF_VERSION`] if unset.
    pub fn esp_idf_version(&self) -> git::Ref {
        self.esp_idf_version
            .clone()
            .unwrap_or_else(|| parse_esp_idf_git_ref(DEFAULT_ESP_IDF_VERSION))
    }

    /// Get the user-specified cmake generator or [`DEFAULT_CMAKE_GENERATOR`] if unset.
    pub fn esp_idf_cmake_generator(&self) -> cmake::Generator {
        self.esp_idf_cmake_generator
            .unwrap_or(DEFAULT_CMAKE_GENERATOR)
    }

    /// Get all component directories of [`Self::extra_components`].
    pub fn extra_component_dirs(&self) -> Result<Vec<PathBuf>> {
        self.extra_components
            .iter()
            .flat_map(|extra_comp| {
                extra_comp
                    .component_dirs
                    .iter()
                    .map(move |v| (v.abspath_relative_to(&extra_comp.manifest_dir), extra_comp))
            })
            .try_fold(Vec::new(), |mut results, (comp_dir, comp)| {
                const CMAKELISTS_TXT: &str = "CMakeLists.txt";
                // Same behavior as the esp-idf cmake component discovery.
                // https://github.com/espressif/esp-idf/blob/36f49f361c001b49c538364056bc5d2d04c6f321/tools/cmake/project.cmake#L202
                // https://github.com/espressif/esp-idf/blob/36f49f361c001b49c538364056bc5d2d04c6f321/tools/cmake/component.cmake#L98
                if !comp_dir.is_dir() {
                    bail!(
                        "extra component dir '{}' specified by crate '{}' does not exist",
                        comp_dir.display(),
                        comp.manifest_dir.display()
                    );
                }

                if comp_dir.join(CMAKELISTS_TXT).exists() {
                    results.push(comp_dir)
                } else {
                    for entry in comp_dir.read_dir()? {
                        let entry = entry?;
                        // Ignore dot-folders.
                        if entry.file_name().to_string_lossy().starts_with('.') {
                            continue;
                        }
                        if entry.path().join(CMAKELISTS_TXT).exists() {
                            results.push(entry.path());
                        }
                    }
                }
                Ok(results)
            })
    }

    /// Get all bindings C headers of extra components where the bindings will be
    /// generated combined with the normal `esp-idf` bindings
    /// (all extra components where [`ExtraComponent::bindings_module`] is [`None`]).
    ///
    /// This method will validate that all returned C header files exist.
    pub fn combined_bindings_headers(&self) -> Result<Vec<PathBuf>> {
        let mut results = Vec::new();
        for comp in &self.extra_components {
            // Skip all extra components with separate bindings.
            if comp.bindings_module.is_some() {
                continue;
            }

            if let Some(header) = &comp.bindings_header {
                let header_path = header.abspath_relative_to(&comp.manifest_dir);

                if !header_path.exists() {
                    bail!(
                        "extra components C header file '{}' specified by crate '{}' does not exist",
                        header_path.display(), comp.manifest_dir.display()
                    );
                }
                results.push(header_path);
            }
        }
        Ok(results)
    }

    /// Get all bindings C headers grouped by the [`ExtraComponent::bindings_module`] name.
    ///
    /// This method will validate that all returned C header files exist and also that the
    /// module name only contains ACII alphanumeric and `_` characters.
    pub fn module_bindings_headers(&self) -> Result<HashMap<&str, Vec<PathBuf>>> {
        let headers = self.extra_components.iter().filter_map(|comp| {
            match (&comp.bindings_header, &comp.bindings_module) {
                (Some(header), Some(module)) => {
                    Some((header.abspath_relative_to(&comp.manifest_dir), module, comp))
                }
                _ => None,
            }
        });
        let mut map = HashMap::<&str, Vec<PathBuf>>::new();

        for (header_path, module_name, comp) in headers {
            if !header_path.exists() {
                bail!(
                    "extra components C header file '{}' specified by crate '{}' does not exist",
                    header_path.display(),
                    comp.manifest_dir.display()
                );
            }
            validate_module_name(module_name, comp)?;
            map.entry(module_name).or_default().push(header_path);
        }
        Ok(map)
    }

    /// Get the configuration from the `package.metadata.esp-idf-sys` object of the root
    /// crate's manifest, and update all options that are [`None`].
    /// Extend [`Self::extra_components`] with all [`ExtraComponent`]s
    /// specified in the root crate's and all direct dependencies' manifest.
    pub fn with_cargo_metadata(&mut self, root: &Package, metadata: &Metadata) -> Result<()> {
        let EspIdfSys {
            v:
                NativeConfig {
                    esp_idf_version,
                    esp_idf_repository,
                    esp_idf_cmake_generator,
                    idf_path,
                    extra_components,
                    esp_idf_components,
                },
        } = EspIdfSys::deserialize(&root.metadata)?;

        set_when_none(&mut self.esp_idf_version, esp_idf_version);
        set_when_none(&mut self.esp_idf_repository, esp_idf_repository);
        set_when_none(&mut self.esp_idf_cmake_generator, esp_idf_cmake_generator);
        set_when_none(&mut self.idf_path, idf_path);
        set_when_none(&mut self.esp_idf_components, esp_idf_components);

        fn make_processor(
            package: &Package,
        ) -> impl Fn(ExtraComponent) -> Option<ExtraComponent> + '_ {
            // Filter empty extra components and set manifest path.
            |mut comp| {
                if comp.bindings_header.is_none() && comp.component_dirs.is_empty() {
                    return None;
                }
                comp.manifest_dir = package
                    .manifest_path
                    .parent()
                    .expect("manifest_path should always have parent")
                    .into();
                Some(comp)
            }
        }

        self.extra_components.extend(
            extra_components
                .into_iter()
                .filter_map(make_processor(root)),
        );

        // Get extra components from all _direct_ dependencies of the root crate.
        let dependencies = metadata
            .resolve
            .as_ref()
            .and_then(|resolve| {
                resolve
                    .nodes
                    .iter()
                    .find(|n| n.id == root.id)
                    .map(|root_node| &root_node.dependencies)
            })
            .into_iter()
            .flatten()
            .flat_map(|id| metadata.packages.iter().find(|p| p.id == *id));

        for dep_package in dependencies {
            let cfg = EspIdfSys::<NativeConfig>::deserialize(&dep_package.metadata)
                .with_context(|| {
                    anyhow!(
                        "failed to parse `package.metadata.esp-idf-sys` of dependency '{}'",
                        &dep_package.name
                    )
                })
                .into_warning();

            if let Some(cfg) = cfg {
                self.extra_components.extend(
                    cfg.v
                        .extra_components
                        .into_iter()
                        .filter_map(make_processor(dep_package)),
                );
            }
        }

        Ok(())
    }
}

/// An extra component to be built, bindings to generate.
///
/// An [`ExtraComponent`] may be used to:
/// - build an extra esp-idf component with [`Self::component_dirs`];
/// - generate the bindings of the header specified by [`Self::bindings_header`].
///
/// Note that it is also possible to only build a component, or only generate bindings.
/// This can be used to generate extra bindings of esp-idf headers.
///
/// ## Example
/// ```toml
/// [[package.metadata.esp-idf-sys.extra-components]]
/// component_dirs = ["rainmaker/components/esp-insights/components", "rainmaker/components"]
/// bindings_header = "bindings.h"
/// bindings_module = "module_name"
/// ```
#[derive(Debug, Deserialize, Clone, Default)]
#[serde(default)]
pub struct ExtraComponent {
    /// A single path or a list of paths to a component directory or directory containing components.
    ///
    /// Each path can be absolute or relative. Relative paths will be relative to the
    /// folder containing the defining `Cargo.toml`.
    ///
    /// **This field is optional.** No component will be built if this field is absent, though
    /// the bindings of the `[Self::bindings_header`] will still be generated.
    #[serde(default, deserialize_with = "parse::value_or_list")]
    pub component_dirs: Vec<PathBuf>,

    /// The path to the C header to generate the bindings with. If this option is absent,
    /// **no** bindings will be generated.
    ///
    /// The path can be absolute or relative. A relative path will be relative to the
    /// folder containing the defining `Cargo.toml`.
    ///
    /// **This field is optional.**
    #[serde(default)]
    pub bindings_header: Option<PathBuf>,

    /// If this field is present, the component bindings will be generated separately from
    /// the `esp-idf` bindings and put into their own module inside the `esp-idf-sys` crate.
    /// Otherwise, if absent, the component bindings will be added to the existing
    /// `esp-idf` bindings (which are available in the crate root).
    ///
    /// To put the bindings into its own module, a separate bindgen instance will generate
    /// the bindings. Note that this will result in duplicate `esp-idf` bindings if the
    /// same `esp-idf` headers that were already processed for the `esp-idf` bindings are
    /// included by the component(s).
    ///
    /// **This field is optional.**
    #[serde(default)]
    pub bindings_module: Option<String>,

    /// Internal field; the path of the directory containing the manifest (`Cargo.toml`)
    /// that defined this [`ExtraComponent`].
    #[serde(skip)]
    pub manifest_dir: PathBuf,
}

mod parse {
    use std::str::FromStr;

    use serde::Deserializer;
    use strum::IntoEnumIterator;

    use super::*;
    pub use crate::config::parse::*;
    use crate::config::utils::ValueOrVec;

    /// Parse a cmake generator, either `default` or one of [`cmake::Generator`].
    pub fn cmake_generator<'d, D: Deserializer<'d>>(
        de: D,
    ) -> Result<Option<cmake::Generator>, D::Error> {
        let gen = Option::<String>::deserialize(de)?
            .map(|s| s.trim().to_lowercase())
            .filter(|s| !s.is_empty());
        let gen = match gen {
            Some(val) => val,
            None => return Ok(None),
        };

        match gen.as_str() {
            "default" => Ok(DEFAULT_CMAKE_GENERATOR),
            other => cmake::Generator::from_str(other).map_err(|_| {
                serde::de::Error::custom(format!(
                    "invalid cmake generator: should be either `default`, or one of [{}]",
                    cmake::Generator::iter()
                        .map(|e| e.into())
                        .collect::<Vec<&'static str>>()
                        .join(", ")
                ))
            }),
        }
        .map(Some)
    }

    pub fn git_ref<'d, D: Deserializer<'d>>(de: D) -> Result<Option<git::Ref>, D::Error> {
        Ok(Option::<String>::deserialize(de)?
            .map(|val| embuild::espidf::parse_esp_idf_git_ref(val.trim())))
    }

    pub fn value_or_list<'d, T, D>(de: D) -> Result<Vec<T>, D::Error>
    where
        D: Deserializer<'d>,
        T: for<'de> Deserialize<'de>,
    {
        Ok(match ValueOrVec::<T>::deserialize(de)? {
            ValueOrVec::Val(val) => vec![val],
            ValueOrVec::Vec(vec) => vec,
        })
    }
}

/// A extra component module name can only contain ASCII alphanumeric and `_` characters.
/// Additionally it must not start with a digit.
pub fn validate_module_name(module_name: &str, comp: &ExtraComponent) -> Result<()> {
    if module_name.is_empty() {
        bail!(
            "extra component module name '{}' specified by crate '{}' cannot be empty",
            module_name,
            comp.manifest_dir.display()
        );
    }

    let mut chars = module_name.chars();
    let first_char = chars.next().unwrap();

    let first_char_valid = first_char.is_ascii_alphabetic() || first_char == '_';
    let other_valid = chars.all(|c| c.is_ascii_alphanumeric() || c == '_');

    if !first_char_valid || !other_valid {
        bail!(
            "extra component module name '{}' specified by crate '{}' can only contain \
             ASCII alphanumeric or `_` characters and must be a valid Rust module name",
            module_name,
            comp.manifest_dir.display()
        );
    }
    Ok(())
}
