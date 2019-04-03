#pragma once

namespace mjcf { namespace detail
{

    template <typename T>
    class const_aware_ptr
    {
    public:
      const_aware_ptr(T* p = nullptr) : _p(p) {}
    
      //void operator=(T* p) { this->_p = p; }
      
      operator T*() { return _p; }
      operator const T*() const { return _p; }
      
      operator bool() { return _p; }
      
      T* operator->() { return _p; }
      const T* operator->() const { return _p; }

    private:
      T* _p;
      
    };
    
}}
