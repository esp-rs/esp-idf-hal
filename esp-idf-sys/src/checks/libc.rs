//! Compile-time checks that ESP-IDF bindgen types and constants match libc definitions.
//! If any of these checks fail, it is likely you need to update `libc` pinned version
//! to the latest one.

use crate as sys;
use compile_fmt::{compile_assert, fmt};

macro_rules! check_constants {
    ($ident:ident) => {
        const _: () = {
            compile_assert!(
                sys::$ident as i64 == libc::$ident as i64,
                "libc/esp-idf-sys constant mismatch for `", stringify!($ident), "`: esp-idf=",
                sys::$ident as i64 => fmt::<i64>(),
                " libc=", libc::$ident as i64 => fmt::<i64>()
            );
        };
    };
}

macro_rules! check_constants_manually {
    ($name:literal, $const1:expr, $const2:expr) => {
        const _: () = {
            compile_assert!(
                $const1 as i64 == $const2 as i64,
                "libc/esp-idf-sys constant mismatch for `", $name, "`: lhs=",
                $const1 as i64 => fmt::<i64>(),
                " rhs=", $const2 as i64 => fmt::<i64>()
            );
        };
    };
}

macro_rules! check_types {
    ($ident:ident) => {
        const _: () = {
            compile_assert!(
                std::mem::size_of::<sys::$ident>() == std::mem::size_of::<libc::$ident>(),
                "libc/esp-idf-sys type mismatch for `", stringify!($ident), "` size: esp-idf=",
                std::mem::size_of::<sys::$ident>() => fmt::<usize>(),
                " libc=", std::mem::size_of::<libc::$ident>() => fmt::<usize>()
            );
            compile_assert!(
                std::mem::align_of::<sys::$ident>() == std::mem::align_of::<libc::$ident>(),
                "libc/esp-idf-sys type mismatch for `", stringify!($ident), "` alignment: esp-idf=",
                std::mem::align_of::<sys::$ident>() => fmt::<usize>(),
                " libc=", std::mem::align_of::<libc::$ident>() => fmt::<usize>()
            );
        };
    };
}

macro_rules! check_types_manually {
    ($name:literal, $size1:expr, $size2:expr, $align1:expr, $align2:expr) => {
        const _: () = {
            compile_assert!(
                $size1 == $size2,
                "libc/esp-idf-sys type mismatch for `", $name, "` size: lhs=",
                $size1 => fmt::<usize>(),
                " rhs=", $size2 => fmt::<usize>()
            );
            compile_assert!(
                $align1 == $align2,
                "libc/esp-idf-sys type mismatch for `", $name, "` alignment: lhs=",
                $align1 => fmt::<usize>(),
                " rhs=", $align2 => fmt::<usize>()
            );
        };
    };
}

// Most of these tests can be generated with from rust-lang/libc tree, with:
// sed -n 's/.*pub type \(.*\) = .*/check_types!(\1);/p' mod.rs | uniq
// sed -n 's/.*pub struct \(.*\) {.*/check_types!(\1);/p' mod.rs | uniq
// sed -n 's/.*pub const \(.*\):.*/check_constants!(\1);/p' mod.rs | uniq
// You should manually check for `cfg` expressions in the source, though.
// Then you should then manually remove any items that don't have a binding in `esp_idf_sys`.
// And finally you should remove duplicates.

// newlib/espidf module (https://github.com/rust-lang/libc/blob/libc-0.2/src/unix/newlib/espidf/mod.rs)
check_types!(clock_t);
// wchar_t is 1 byte on xtensa (esp32/s2/s3) before IDF v5.4 vs 4 bytes in libc.
#[cfg(any(not(any(esp32, esp32s2, esp32s3)), esp_idf_version_at_least_5_4_0))]
check_types!(wchar_t);
check_types!(cmsghdr);
check_types!(msghdr);
//check_types!(sockaddr_un); // No binding
check_types!(sockaddr);
check_types!(sockaddr_in6);
check_types!(sockaddr_in);
check_types!(sockaddr_storage);
//check_constants!(AF_UNIX); // No binding
check_constants!(AF_INET6);
//check_constants!(FIONBIO); // No binding
check_constants!(POLLIN);
check_constants!(POLLRDNORM);
check_constants!(POLLRDBAND);
check_constants!(POLLPRI);
check_constants!(POLLOUT);
check_constants!(POLLWRNORM);
check_constants!(POLLWRBAND);
check_constants!(POLLERR);
check_constants!(POLLHUP);
check_constants!(SOL_SOCKET);
check_constants!(MSG_OOB);
check_constants!(MSG_PEEK);
check_constants!(MSG_DONTWAIT);
check_constants!(MSG_DONTROUTE);
check_constants!(MSG_WAITALL);
check_constants!(MSG_MORE);
check_constants!(MSG_NOSIGNAL);
check_constants!(MSG_TRUNC);
check_constants!(MSG_CTRUNC);
check_constants!(MSG_EOR);
check_constants!(PTHREAD_STACK_MIN);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGABRT);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGFPE);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGILL);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGINT);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGSEGV);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGTERM);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGHUP);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGQUIT);
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(NSIG);
check_constants!(SOMAXCONN);

// newlib module (https://github.com/rust-lang/libc/blob/libc-0.2/src/unix/newlib/mod.rs)
check_types!(blkcnt_t);
check_types!(blksize_t);
check_types!(clockid_t);
check_types!(dev_t);
check_types!(ino_t);
check_types!(off_t);
check_types!(fsblkcnt_t);
check_types!(fsfilcnt_t);
check_types!(id_t);
check_types!(key_t);
check_types_manually!(
    "loff_t",
    std::mem::size_of::<sys::__loff_t>(),
    std::mem::size_of::<libc::loff_t>(),
    std::mem::align_of::<sys::__loff_t>(),
    std::mem::align_of::<libc::loff_t>()
);
check_types!(mode_t);
check_types!(nfds_t);
check_types!(nlink_t);
check_types!(pthread_t);
check_types!(pthread_key_t);
//check_types!(rlim_t); // No binding
check_types!(sa_family_t);
check_types!(socklen_t);
#[cfg(not(esp_idf_version_at_least_6_0_0))]
check_types!(speed_t);
check_types!(suseconds_t);
#[cfg(not(esp_idf_version_at_least_6_0_0))]
check_types!(tcflag_t);
check_types!(useconds_t);
check_types!(time_t);
// structs
check_types!(addrinfo);
check_types!(ip_mreq);
check_types!(in_addr);
//check_types!(lconv); // No binding
check_types!(tm);
#[cfg(not(esp_idf_libc_picolibc))]
check_types!(sigaction);
#[cfg(not(esp_idf_libc_picolibc))]
check_types!(stack_t);
#[cfg(esp_idf_version_at_least_5_0_0)]
check_types!(fd_set);
//check_types!(passwd); // No binding
#[cfg(not(esp_idf_version_at_least_6_0_0))]
check_types!(termios);
//check_types!(sem_t); // No binding
//check_types!(utsname); // No binding
//check_types!(cpu_set_t); // No binding
/* TODO: Size/alignment mismatches, to be fixed, or at least carefully investigated
check_types!(pthread_attr_t);
check_types!(pthread_rwlockattr_t);
check_types!(pthread_mutex_t);
check_types!(pthread_rwlock_t);
check_types!(pthread_mutexattr_t);
check_types!(pthread_cond_t);
check_types!(pthread_condattr_t);*/
#[cfg(not(esp_idf_version_at_least_6_0_0))]
check_constants!(NCCS);
check_constants!(PTHREAD_MUTEX_NORMAL);
check_constants!(PTHREAD_MUTEX_RECURSIVE);
check_constants!(PTHREAD_MUTEX_ERRORCHECK);
check_constants!(FD_SETSIZE);
check_constants!(EPERM);
check_constants!(ENOENT);
check_constants!(ESRCH);
check_constants!(EINTR);
check_constants!(EIO);
check_constants!(ENXIO);
check_constants!(E2BIG);
check_constants!(ENOEXEC);
check_constants!(EBADF);
check_constants!(ECHILD);
check_constants!(EAGAIN);
check_constants!(ENOMEM);
check_constants!(EACCES);
check_constants!(EFAULT);
check_constants!(EBUSY);
check_constants!(EEXIST);
check_constants!(EXDEV);
check_constants!(ENODEV);
check_constants!(ENOTDIR);
check_constants!(EISDIR);
check_constants!(EINVAL);
check_constants!(ENFILE);
check_constants!(EMFILE);
check_constants!(ENOTTY);
check_constants!(ETXTBSY);
check_constants!(EFBIG);
check_constants!(ENOSPC);
check_constants!(ESPIPE);
check_constants!(EROFS);
check_constants!(EMLINK);
check_constants!(EPIPE);
check_constants!(EDOM);
check_constants!(ERANGE);
check_constants!(ENOMSG);
check_constants!(EIDRM);
check_constants!(EDEADLK);
check_constants!(ENOLCK);
check_constants!(ENOSTR);
check_constants!(ENODATA);
check_constants!(ETIME);
check_constants!(ENOSR);
check_constants!(ENOLINK);
check_constants!(EPROTO);
check_constants!(EMULTIHOP);
check_constants!(EBADMSG);
check_constants!(EFTYPE);
check_constants!(ENOSYS);
check_constants!(ENOTEMPTY);
check_constants!(ENAMETOOLONG);
check_constants!(ELOOP);
check_constants!(EOPNOTSUPP);
check_constants!(EPFNOSUPPORT);
check_constants!(ECONNRESET);
check_constants!(ENOBUFS);
check_constants!(EAFNOSUPPORT);
check_constants!(EPROTOTYPE);
check_constants!(ENOTSOCK);
check_constants!(ENOPROTOOPT);
check_constants!(ECONNREFUSED);
check_constants!(EADDRINUSE);
check_constants!(ECONNABORTED);
check_constants!(ENETUNREACH);
check_constants!(ENETDOWN);
check_constants!(ETIMEDOUT);
check_constants!(EHOSTDOWN);
check_constants!(EHOSTUNREACH);
check_constants!(EINPROGRESS);
check_constants!(EALREADY);
check_constants!(EDESTADDRREQ);
check_constants!(EMSGSIZE);
check_constants!(EPROTONOSUPPORT);
check_constants!(EADDRNOTAVAIL);
check_constants!(ENETRESET);
check_constants!(EISCONN);
check_constants!(ENOTCONN);
check_constants!(ETOOMANYREFS);
check_constants!(EDQUOT);
check_constants!(ESTALE);
check_constants!(ENOTSUP);
check_constants!(EILSEQ);
check_constants!(EOVERFLOW);
check_constants!(ECANCELED);
check_constants!(ENOTRECOVERABLE);
check_constants!(EOWNERDEAD);
check_constants!(EWOULDBLOCK);
check_constants!(F_DUPFD);
check_constants!(F_GETFD);
check_constants!(F_SETFD);
check_constants!(F_GETFL);
check_constants!(F_SETFL);
check_constants!(F_GETOWN);
check_constants!(F_SETOWN);
check_constants!(F_GETLK);
check_constants!(F_SETLK);
check_constants!(F_SETLKW);
check_constants!(F_RGETLK);
check_constants!(F_RSETLK);
check_constants!(F_CNVT);
check_constants!(F_RSETLKW);
check_constants!(F_DUPFD_CLOEXEC);
check_constants!(O_RDONLY);
check_constants!(O_WRONLY);
check_constants!(O_RDWR);
// TODO: picolibc incompatibility, see https://github.com/esp-rs/esp-idf-sys/issues/410
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(O_APPEND);
// TODO: picolibc incompatibility, see https://github.com/esp-rs/esp-idf-sys/issues/410
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(O_CREAT);
// TODO: picolibc incompatibility, see https://github.com/esp-rs/esp-idf-sys/issues/410
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(O_TRUNC);
check_constants!(O_EXCL);
check_constants!(O_SYNC);
check_constants!(O_NONBLOCK);
//check_constants!(O_ACCMODE); // No binding
check_constants!(O_CLOEXEC);
//check_constants!(RTLD_LAZY); // No binding
check_constants!(SEEK_SET);
check_constants!(SEEK_CUR);
check_constants!(SEEK_END);
//check_constants!(FIOCLEX); // No binding
//check_constants!(FIONCLEX); // No binding
check_constants!(S_BLKSIZE);
check_constants!(S_IREAD);
check_constants!(S_IWRITE);
check_constants!(S_IEXEC);
check_constants!(S_ENFMT);
check_constants!(S_IFMT);
check_constants!(S_IFDIR);
check_constants!(S_IFCHR);
check_constants!(S_IFBLK);
check_constants!(S_IFREG);
check_constants!(S_IFLNK);
check_constants!(S_IFSOCK);
check_constants!(S_IFIFO);
check_constants!(S_IRUSR);
check_constants!(S_IWUSR);
check_constants!(S_IXUSR);
check_constants!(S_IRGRP);
check_constants!(S_IWGRP);
check_constants!(S_IXGRP);
check_constants!(S_IROTH);
check_constants!(S_IWOTH);
check_constants!(S_IXOTH);
//check_constants!(SOL_TCP); // No binding
check_constants!(PF_UNSPEC);
check_constants!(PF_INET);
check_constants!(PF_INET6);
check_constants!(AF_UNSPEC);
check_constants!(AF_INET);
//check_constants!(CLOCK_REALTIME); // No binding
//check_constants!(CLOCK_MONOTONIC); // No binding
//check_constants!(CLOCK_BOOTTIME); // No binding
check_constants!(SOCK_STREAM);
check_constants!(SOCK_DGRAM);
check_constants!(SHUT_RD);
check_constants!(SHUT_WR);
check_constants!(SHUT_RDWR);
/* No bindings
check_constants!(SO_BINTIME);
check_constants!(SO_NO_OFFLOAD);
check_constants!(SO_NO_DDP);
check_constants!(SO_REUSEPORT_LB);
check_constants!(SO_LABEL);
check_constants!(SO_PEERLABEL);
check_constants!(SO_LISTENQLIMIT);
check_constants!(SO_LISTENQLEN);
check_constants!(SO_LISTENINCQLEN);
check_constants!(SO_SETFIB);
check_constants!(SO_USER_COOKIE);
check_constants!(SO_PROTOCOL);
check_constants!(SO_PROTOTYPE);
check_constants!(SO_VENDOR);
*/
check_constants!(SO_DEBUG);
check_constants!(SO_ACCEPTCONN);
check_constants!(SO_REUSEADDR);
check_constants!(SO_KEEPALIVE);
check_constants!(SO_DONTROUTE);
check_constants!(SO_BROADCAST);
check_constants!(SO_USELOOPBACK);
check_constants!(SO_LINGER);
check_constants!(SO_OOBINLINE);
check_constants!(SO_REUSEPORT);
//check_constants!(SO_TIMESTAMP); // No binding
//check_constants!(SO_NOSIGPIPE); // No binding
//check_constants!(SO_ACCEPTFILTER); // No binding
check_constants!(SO_SNDBUF);
check_constants!(SO_RCVBUF);
check_constants!(SO_SNDLOWAT);
check_constants!(SO_RCVLOWAT);
check_constants!(SO_SNDTIMEO);
check_constants!(SO_RCVTIMEO);
check_constants!(SO_ERROR);
check_constants!(SO_TYPE);
check_constants_manually!("SOCK_CLOEXEC", sys::O_CLOEXEC, libc::SOCK_CLOEXEC);
check_constants!(INET_ADDRSTRLEN);
/* No bindings
check_constants!(IFF_UP);
check_constants!(IFF_BROADCAST);
check_constants!(IFF_DEBUG);
check_constants!(IFF_LOOPBACK);
check_constants!(IFF_POINTOPOINT);
check_constants!(IFF_NOTRAILERS);
check_constants!(IFF_RUNNING);
check_constants!(IFF_NOARP);
check_constants!(IFF_PROMISC);
check_constants!(IFF_ALLMULTI);
check_constants!(IFF_OACTIVE);
check_constants!(IFF_SIMPLEX);
check_constants!(IFF_LINK0);
check_constants!(IFF_LINK1);
check_constants!(IFF_LINK2);
check_constants!(IFF_ALTPHYS);
check_constants!(IFF_MULTICAST);
*/
check_constants!(TCP_NODELAY);
//check_constants!(TCP_MAXSEG); // No binding
//check_constants!(TCP_NOPUSH); // No binding
//check_constants!(TCP_NOOPT); // No binding
check_constants!(TCP_KEEPIDLE);
check_constants!(TCP_KEEPINTVL);
check_constants!(TCP_KEEPCNT);
check_constants!(IP_TOS);
check_constants!(IP_TTL);
check_constants!(IP_MULTICAST_IF);
check_constants!(IP_MULTICAST_TTL);
check_constants!(IP_MULTICAST_LOOP);
check_constants!(IP_ADD_MEMBERSHIP);
check_constants!(IP_DROP_MEMBERSHIP);
check_constants!(IPV6_UNICAST_HOPS);
check_constants!(IPV6_MULTICAST_IF);
check_constants!(IPV6_MULTICAST_HOPS);
check_constants!(IPV6_MULTICAST_LOOP);
check_constants!(IPV6_V6ONLY);
check_constants!(IPV6_JOIN_GROUP);
check_constants!(IPV6_LEAVE_GROUP);
check_constants!(IPV6_ADD_MEMBERSHIP);
check_constants!(IPV6_DROP_MEMBERSHIP);
check_constants!(HOST_NOT_FOUND);
check_constants!(NO_DATA);
check_constants!(NO_RECOVERY);
check_constants!(TRY_AGAIN);
//check_constants!(NO_ADDRESS); // No binding
check_constants!(AI_PASSIVE);
check_constants!(AI_CANONNAME);
check_constants!(AI_NUMERICHOST);
check_constants!(AI_NUMERICSERV);
check_constants!(AI_ADDRCONFIG);
check_constants!(NI_MAXHOST);
check_constants!(NI_MAXSERV);
//check_constants!(NI_NOFQDN); // No binding
//check_constants!(NI_NUMERICHOST); // No binding
//check_constants!(NI_NAMEREQD); // No binding
check_constants!(NI_NUMERICSERV);
check_constants!(NI_DGRAM);
check_constants!(EAI_FAMILY);
check_constants!(EAI_MEMORY);
check_constants!(EAI_NONAME);
check_constants!(EAI_SOCKTYPE);
check_constants!(EXIT_SUCCESS);
check_constants!(EXIT_FAILURE);
//check_constants!(PRIO_PROCESS); // No binding
//check_constants!(PRIO_PGRP); // No binding
//check_constants!(PRIO_USER); // No binding

// unix module (https://github.com/rust-lang/libc/blob/libc-0.2/src/unix/mod.rs)
check_types!(intmax_t);
check_types!(uintmax_t);
/* No bindings
check_types!(size_t);
check_types!(ptrdiff_t);
check_types!(intptr_t);
check_types!(uintptr_t);
check_types!(ssize_t);
*/
check_types!(pid_t);
check_types!(in_addr_t);
check_types!(in_port_t);
//check_types!(sighandler_t); // No binding
#[cfg(not(esp_idf_version_at_least_6_0_0))]
check_types!(cc_t);
check_types!(uid_t);
check_types!(gid_t);
check_types!(locale_t);
// structs
//check_types!(group); // No binding
check_types!(utimbuf);
check_types!(timeval);
check_types!(timespec);
//check_types!(rlimit); // No binding
//check_types!(rusage); // No binding
check_types!(ipv6_mreq);
check_types!(hostent);
check_types!(iovec);
check_types!(pollfd);
//check_types!(winsize); // No binding
check_types!(linger);
#[cfg(not(esp_idf_libc_picolibc))]
check_types!(sigval);
check_types!(itimerval);
//#[cfg(not(esp_idf_libc_picolibc))]
// TODO: Mismatch for type `tms` size: esp-idf=0 libc=16
//check_types!(tms);
//check_types!(servent); // No binding
//check_types!(protoent); // No binding
check_types!(in6_addr);
/* No bindings
check_constants!(INT_MIN);
check_constants!(INT_MAX);
check_constants!(SIG_DFL);
check_constants!(SIG_IGN);
check_constants!(SIG_ERR);
*/
check_constants!(DT_UNKNOWN);
/* TODO: Fixed upstream https://github.com/rust-lang/libc/pull/5034, uncomment after libc release
#[cfg(esp_idf_libc_picolibc)]
check_constants!(DT_FIFO);
#[cfg(esp_idf_libc_picolibc)]
check_constants!(DT_CHR);
check_constants!(DT_DIR); */
#[cfg(esp_idf_libc_picolibc)]
check_constants!(DT_BLK);
/* TODO: Fixed upstream https://github.com/rust-lang/libc/pull/5034, uncomment after libc release
check_constants!(DT_REG); */
#[cfg(esp_idf_libc_picolibc)]
check_constants!(DT_LNK);
#[cfg(esp_idf_libc_picolibc)]
check_constants!(DT_SOCK);
check_constants!(FD_CLOEXEC);
//check_constants!(USRQUOTA); // No binding
//check_constants!(GRPQUOTA); // No binding
#[cfg(not(esp_idf_libc_picolibc))]
check_constants!(SIGIOT);
check_constants!(S_ISUID);
check_constants!(S_ISGID);
check_constants!(S_ISVTX);
// We pickup a value from SDK's lwip instead of the libc header => likely harmless.
/* TODO: Double-check these
check_constants!(IF_NAMESIZE);
check_constants!(IFNAMSIZ);*/
/* No bindings
check_constants!(LOG_EMERG);
check_constants!(LOG_ALERT);
check_constants!(LOG_CRIT);
check_constants!(LOG_ERR);
check_constants!(LOG_WARNING);
check_constants!(LOG_NOTICE);
check_constants!(LOG_INFO);
check_constants!(LOG_DEBUG);
check_constants!(LOG_KERN);
check_constants!(LOG_USER);
check_constants!(LOG_MAIL);
check_constants!(LOG_DAEMON);
check_constants!(LOG_AUTH);
check_constants!(LOG_SYSLOG);
check_constants!(LOG_LPR);
check_constants!(LOG_NEWS);
check_constants!(LOG_UUCP);
check_constants!(LOG_LOCAL0);
check_constants!(LOG_LOCAL1);
check_constants!(LOG_LOCAL2);
check_constants!(LOG_LOCAL3);
check_constants!(LOG_LOCAL4);
check_constants!(LOG_LOCAL5);
check_constants!(LOG_LOCAL6);
check_constants!(LOG_LOCAL7);
check_constants!(LOG_PID);
check_constants!(LOG_CONS);
check_constants!(LOG_ODELAY);
check_constants!(LOG_NDELAY);
check_constants!(LOG_NOWAIT);
check_constants!(LOG_PRIMASK);
check_constants!(LOG_FACMASK);
check_constants!(PRIO_MIN);
check_constants!(PRIO_MAX);
*/
check_constants!(IPPROTO_ICMP);
check_constants!(IPPROTO_ICMPV6);
check_constants!(IPPROTO_TCP);
check_constants!(IPPROTO_UDP);
check_constants!(IPPROTO_IP);
check_constants!(IPPROTO_IPV6);
/* No bindings
check_constants!(INADDR_LOOPBACK);
check_constants!(INADDR_ANY);
check_constants!(INADDR_BROADCAST);
check_constants!(INADDR_NONE);
check_constants!(IN6ADDR_LOOPBACK_INIT);
check_constants!(IN6ADDR_ANY_INIT);
check_constants!(ARPOP_REQUEST);
check_constants!(ARPOP_REPLY);
check_constants!(ATF_COM);
check_constants!(ATF_PERM);
check_constants!(ATF_PUBL);
check_constants!(ATF_USETRAILERS);
*/

// misc
check_constants!(STDIN_FILENO);
check_constants!(STDOUT_FILENO);
check_constants!(STDERR_FILENO);
