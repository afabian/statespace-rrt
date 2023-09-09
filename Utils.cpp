#include "Utils.h"

#include <cstring>
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

    mkpath(dirname(strdupa(dir)), mode);

    return mkdir(dir, mode);
}