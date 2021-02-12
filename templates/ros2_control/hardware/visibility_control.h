$LICENSE$

#ifndef $PACKAGE_NAME$__VISIBILITY_CONTROL_H_
#define $PACKAGE_NAME$__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define $PACKAGE_NAME$_EXPORT __attribute__((dllexport))
#define $PACKAGE_NAME$_IMPORT __attribute__((dllimport))
#else
#define $PACKAGE_NAME$_EXPORT __declspec(dllexport)
#define $PACKAGE_NAME$_IMPORT __declspec(dllimport)
#endif
#ifdef $PACKAGE_NAME$_BUILDING_DLL
#define $PACKAGE_NAME$_PUBLIC $PACKAGE_NAME$_EXPORT
#else
#define $PACKAGE_NAME$_PUBLIC $PACKAGE_NAME$_IMPORT
#endif
#define $PACKAGE_NAME$_PUBLIC_TYPE $PACKAGE_NAME$_PUBLIC
#define $PACKAGE_NAME$_LOCAL
#else
#define $PACKAGE_NAME$_EXPORT __attribute__((visibility("default")))
#define $PACKAGE_NAME$_IMPORT
#if __GNUC__ >= 4
#define $PACKAGE_NAME$_PUBLIC __attribute__((visibility("default")))
#define $PACKAGE_NAME$_LOCAL __attribute__((visibility("hidden")))
#else
#define $PACKAGE_NAME$_PUBLIC
#define $PACKAGE_NAME$_LOCAL
#endif
#define $PACKAGE_NAME$_PUBLIC_TYPE
#endif

#endif  // $PACKAGE_NAME$__VISIBILITY_CONTROL_H_
