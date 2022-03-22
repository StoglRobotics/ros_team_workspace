$LICENSE$

#ifndef DUMMY_PACKAGE_NAME__VISIBILITY_CONTROL_H_
#define DUMMY_PACKAGE_NAME__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DUMMY_PACKAGE_NAME_EXPORT __attribute__((dllexport))
#define DUMMY_PACKAGE_NAME_IMPORT __attribute__((dllimport))
#else
#define DUMMY_PACKAGE_NAME_EXPORT __declspec(dllexport)
#define DUMMY_PACKAGE_NAME_IMPORT __declspec(dllimport)
#endif
#ifdef DUMMY_PACKAGE_NAME_BUILDING_DLL
#define DUMMY_PACKAGE_NAME_PUBLIC DUMMY_PACKAGE_NAME_EXPORT
#else
#define DUMMY_PACKAGE_NAME_PUBLIC DUMMY_PACKAGE_NAME_IMPORT
#endif
#define DUMMY_PACKAGE_NAME_PUBLIC_TYPE DUMMY_PACKAGE_NAME_PUBLIC
#define DUMMY_PACKAGE_NAME_LOCAL
#else
#define DUMMY_PACKAGE_NAME_EXPORT __attribute__((visibility("default")))
#define DUMMY_PACKAGE_NAME_IMPORT
#if __GNUC__ >= 4
#define DUMMY_PACKAGE_NAME_PUBLIC __attribute__((visibility("default")))
#define DUMMY_PACKAGE_NAME_LOCAL __attribute__((visibility("hidden")))
#else
#define DUMMY_PACKAGE_NAME_PUBLIC
#define DUMMY_PACKAGE_NAME_LOCAL
#endif
#define DUMMY_PACKAGE_NAME_PUBLIC_TYPE
#endif

#endif  // DUMMY_PACKAGE_NAME__VISIBILITY_CONTROL_H_
