#ifndef ARMORDETECTION_UTILS_H
#define ARMORDETECTION_UTILS_H

#include <algorithm>
#include <cmath>
#include <functional>
#include <forward_list>
#include <string>
#include <vector>

namespace HYY{ 

namespace utils {

    bool fileExists(const std::string &filename);

    std::forward_list<std::string> getFilesFromFolder(const std::string &directory, const std::string &ext);

    inline int sqr(int n) {
        return n * n;
    }

    inline float sqr(float n) {
        return n * n;
    }

    inline int round(double n) {
        return (int)std::lround(n);
    }

    inline int round(float n) {
        return (int)std::lroundf(n);
    }

    template<typename SrcElem, typename... Alloc, typename Product>
    std::forward_list<Product> map(const std::forward_list<SrcElem, Alloc...> &collection, const std::function<Product(SrcElem)> &mapper) {
        std::forward_list<Product> ans;
        for (auto &e: collection)
            ans.push_front(mapper(e));
        return ans;
    };

    template<typename SrcElem, template<typename, typename...> class Collection, typename... Alloc, typename Product>
    std::vector<Product> map(const Collection<SrcElem, Alloc...> &collection, const std::function<Product(SrcElem)> &mapper) {
        std::vector<Product> ans(collection.size());
        std::transform(collection.begin(), collection.end(), ans.begin(), mapper);
        return ans;
    };

    template<typename Elem, template<typename, typename...> class Collection, typename... Alloc, typename Predicate>
    std::forward_list<Elem> filter(const Collection<Elem, Alloc...> &collection, Predicate predicate) {
        std::forward_list<Elem> ans;
        for (auto cp = collection.begin(); cp != collection.end(); ++cp)
            if (predicate(*cp))
                ans.push_front(*cp);
        return ans;
    };

}

} // namespace HYY


#endif //ARMORDETECTION_UTILS_H
