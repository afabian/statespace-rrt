#include "Utils.h"

#include <string>
#include <cerrno>
#include <libgen.h>

int mkpath(const char* dir, mode_t mode) {
    struct stat sb;

    if (!dir) {
        errno = EINVAL;
        return 1;
    }

    if (!stat(dir, &sb))
        return 0;

    char dircopy[1024] = "";
    strcpy(dircopy, dir);
    mkpath(dirname(dircopy), mode);

#ifdef __MINGW32__
    return mkdir(dir);
#else
    return mkdir(dir, mode);
#endif

}