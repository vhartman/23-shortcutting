#include <vector>

#ifndef CORPUT_H
#define CORPUT_H

class Corput{
  public:
    Corput(){
      for (uint i=0; i<100000; ++i){
        get(i);
      }
    };

    static constexpr bool caching{true};

    inline double get(uint n){
      if (caching && cache.size() > n){
        return cache[n];
      }

      double q=0;
      double bk=(double)1/base;

      while (n > 0) {
        q += (n % base)*bk;
        n /= base;
        bk /= base;
      }

      if (caching){
        cache.push_back(q);
      }

      return q;
    }

    void reset(){
      cache.clear();
    }

  private:
    std::vector<double> cache;
    const uint base = 2;
};

inline Corput corput;

#endif
