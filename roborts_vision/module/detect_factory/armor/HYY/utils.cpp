#include "utils.h"
#include <dirent.h>
#include <fstream>
#include <stdexcept>

using std::forward_list;
using std::string;

namespace HYY{ 

bool _hasExt(const string &name, const string &ext) {
    string::const_reverse_iterator ep, fp;
    for (ep = ext.crbegin(), fp = name.crbegin(); ep != ext.crend() && fp != name.crend(); ++ep, ++fp)
        if (*ep != *fp) return false;
    return ep == ext.crend() && fp != name.crend() && *fp == '.';
}

namespace utils {

    bool fileExists(const std::string &filename) {
        return std::ifstream(filename).good();
    }

    forward_list<string> getFilesFromFolder(const string &directory, const string &ext) {
        forward_list<string> files;
        struct dirent *dp;
        DIR *dir = opendir(directory.c_str());
        if (dir == nullptr)
            throw std::invalid_argument("invalid directory");

        while ((dp = readdir(dir)) != nullptr) {
            if (dp->d_type == 8 && _hasExt(dp->d_name, ext)) {
                string completeFilename = directory
                                          #ifdef __unix__
                                          + (*directory.crbegin() == '/' ? "" : "/")
                                          #else
                                          + (*directory.crbegin() == '\\' ? "" : "\\")
                                          #endif
                                          + dp->d_name;
                files.push_front(completeFilename);
            }
        }
        closedir(dir);
        return files;
    }

}

} // namespace HYY