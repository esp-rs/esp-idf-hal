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

pub const DEFAULT_ESP_IDF_VERSION: &str = "v4.4.1";
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
    idf_path: Option<PathBuf>,

    /// Additional components to build and maybe generate bindings for.
    ///
    /// Can be specified in the root crate's `package.metadata.esp-idf-sys` and all direct
    /// dependencies'.
    #[serde(alias = "extra-components")]
    pub extra_components: Vec<ExtraComponent>,
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

    pub fn with_cargo_metadata(&mut self, root: &Package, metadata: &Metadata) -> Result<()> {
        let EspIdfSys {
            v:
                NativeConfig {
                    esp_idf_version,
                    esp_idf_repository,
                    esp_idf_cmake_generator,
                    idf_path,
                    extra_components,
                },
        } = EspIdfSys::deserialize(&root.metadata)?;

        set_when_none(&mut self.esp_idf_version, esp_idf_version);
        set_when_none(&mut self.esp_idf_repository, esp_idf_repository);
        set_when_none(&mut self.esp_idf_cmake_generator, esp_idf_cmake_generator);
        set_when_none(&mut self.idf_path, idf_path);

        fn make_processor<'a>(
            package: &'a Package,
        ) -> impl Fn(ExtraComponent) -> Option<ExtraComponent> + 'a {
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

#[derive(Debug, Deserialize, Clone, Default)]
#[serde(default)]
pub struct ExtraComponent {
    /// A single path or a list of paths to a component directory, or directory containing components.
    ///
    /// Each path can be absolute or relative. Relative paths will be relative to the
    /// folder containg the defining `Cargo.toml`.
    ///
    /// ## Examples
    /// Multiple component dirs:
    /// ```toml
    /// [[package.metadata.esp-idf-sys.extra-components]]
    /// component_dirs = ["rainmaker/components/esp-insights/components", "rainmaker/components"]
    /// ```
    ///
    /// A single dir that contains one or more esp-idf components:
    /// ```toml
    /// [[package.metadata.esp-idf-sys.extra-components]]
    /// component_dirs = "extra_components"
    /// ```
    ///
    /// A single component:
    /// ```toml
    /// [[package.metadata.esp-idf-sys.extra-components]]
    /// component_dirs = "my_component"
    /// ```
    #[serde(deserialize_with = "parse::value_or_list")]
    pub component_dirs: Vec<PathBuf>,

    /// The path to the C header to generate the bindings with. If this option is absent,
    /// **no** bindings will be generated.
    #[serde(default)]
    pub bindings_header: Option<PathBuf>,

    /// If this option is present, the component bindings will be generated separately from
    /// the `esp-idf` bindings and put into their own module inside the `esp-idf-sys` crate.
    /// Otherwise, if absent, the component bindings will be added to the existing
    /// `esp-idf` bindings (which are available in the crate root).
    ///
    /// To put the bindings into its own module, a separate bindgen instanace will generate
    /// the bindings. Note that this will result in duplicate `esp-idf` bindings if the same
    /// `esp-idf` headers are included by the component(s) that were already processed for
    /// the `esp-idf` bindings.
    ///
    /// Optional
    #[serde(default)]
    pub bindings_module: Option<String>,

    #[serde(skip)]
    pub manifest_dir: PathBuf,
}

mod parse {
    use std::str::FromStr;

    use embuild::{cmake, git};
    use serde::{Deserialize, Deserializer};
    use strum::IntoEnumIterator;

    use super::DEFAULT_CMAKE_GENERATOR;
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
