#pragma once

#include "common/TypeList.hpp"

#include <vector>
#include <iostream>
#include <memory>

// adapted from this StackOverflow answer: https://stackoverflow.com/a/53112843

/** A class that stores a heterogeneous collection of vector types (i.e. vectors that each store different types).
 * The types stored are determined by the template parameters, meaning that it is determined at compile time.
 * 
 * Uses CRTP inheritance to recursively add a private member vector variable for each type.
 */
template<class L, class... R> class VariadicVectorContainer;

template<class L>
class VariadicVectorContainer<L>
{
    protected:
    const std::vector<L>& _get() const
    {
        return _vec;
    }

    std::vector<L>& _get()
    {
        return _vec;
    }

    size_t _size() const
    {
        return _vec.size();
    }

    void _resize(int size)
    {
        _vec.resize(size);
    }

    void _reserve(int size)
    {
        _vec.reserve(size);
    }

    void _push_back(const L& elem)
    {
        _vec.push_back(elem);
    }

    void _push_back(L&& elem)
    {
        _vec.push_back(std::move(elem));
    }

    template<class ...Args>
    L& _emplace_back(Args&&... args)
    {
        return _vec.emplace_back(std::forward<Args>(args)...);
    } 

    L& _set(int index, const L& elem)
    {
        _vec[index] = elem;
        return _vec[index];
    }

    L& _set(int index, L&& elem)
    {
        _vec[index] = std::move(elem);
        return _vec[index];
    }

    void _clear()
    {
        _vec.clear();
    }

    // internal method for visiting elements
    template<typename Visitor>
    void _for_each_element(Visitor&& visitor) const
    {
        for (const auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    template<typename Visitor>
    void _for_each_element(Visitor&& visitor)
    {
        for (auto& elem : _vec)
        {
            visitor(elem);
        }
    }

    template<typename Visitor>
    void _for_each_element_indexed(Visitor&& visitor) const
    {
        for (size_t i = 0; i < _vec.size(); ++i)
        {
            visitor(_vec[i], i);
        }
    }

    template<typename Visitor>
    void _for_each_element_indexed(Visitor&& visitor)
    {
        for (size_t i = 0; i < _vec.size(); ++i)
        {
            visitor(_vec[i], i);
        }
    }

    private:
    std::vector<L> _vec;
};

template<class L, class... R>
class VariadicVectorContainer : public VariadicVectorContainer<L>, public VariadicVectorContainer<R...>
{
    public:
    size_t size() const
    {
        return _size_helper<L, R...>();
    }

    void clear()
    {
        _clear_helper<L, R...>();
    }

    template<class T>
    const std::vector<T>& get() const
    {
        return this->VariadicVectorContainer<T>::_get();
    }

    template<class T>
    std::vector<T>& get()
    {
        return this->VariadicVectorContainer<T>::_get();
    }

    template<class T>
    void push_back(const T& elem)
    {
        return this->VariadicVectorContainer<T>::_push_back(elem);
    }

    template<class T>
    void push_back(T&& elem)
    {
        return this->VariadicVectorContainer<T>::_push_back(std::move(elem));
    }

    template<class T, class ...Args>
    T& emplace_back(Args&&... args)
    {
        return this->VariadicVectorContainer<T>::_emplace_back(std::forward<Args>(args)...);
    }

    template<class T>
    void resize(int size)
    {
        return this->VariadicVectorContainer<T>::_resize(size);
    }

    template<class T>
    void reserve(int size)
    {
        return this->VariadicVectorContainer<T>::_reserve(size);
    }

    template<class T>
    T& set(int index, const T& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, elem);
    }

    template<class T>
    T& set(int index, T&& elem)
    {
        return this->VariadicVectorContainer<T>::_set(index, std::move(elem));
    }

    template<class T>
    size_t size() const
    {
        return this->VariadicVectorContainer<T>::_size();
    }

    template<class T>
    void clear()
    {
        return this->VariadicVectorContainer<T>::_clear();
    }

    // visit all elements in a subset of types - only enable this overload if sizeof(Ts) > 0
    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(Visitor&& visitor) const
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(Visitor&& visitor)
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(TypeList<Ts...>, Visitor&& visitor) const
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    template<typename... Ts, typename Visitor>
    std::enable_if_t<(sizeof...(Ts) > 0), void>
    for_each_element(TypeList<Ts...>, Visitor&& visitor)
    {
        _visit_elements<Ts...>(std::forward<Visitor>(visitor));
    }

    // visit all elements across all vectors
    template<typename Visitor>
    void for_each_element(Visitor&& visitor) const
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    template<typename Visitor>
    void for_each_element(Visitor&& visitor)
    {
        _visit_elements<L, R...>(std::forward<Visitor>(visitor));
    }

    template<typename Visitor>
    void for_each_element_indexed(Visitor&& visitor) const
    {
        _visit_elements_indexed<L, R...>(std::forward<Visitor>(visitor));
    }

    template<typename Visitor>
    void for_each_element_indexed(Visitor&& visitor)
    {
        _visit_elements_indexed<L, R...>(std::forward<Visitor>(visitor));
    }

    private:
    
    // recursive implementation to visit elements of all types
    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor) const
    {
        this->VariadicVectorContainer<T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements(Visitor&& visitor)
    {
        this->VariadicVectorContainer<T>::_for_each_element(visitor);

        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements_indexed(Visitor&& visitor) const
    {
        this->VariadicVectorContainer<T>::_for_each_element_indexed(visitor);
        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements_indexed<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts, typename Visitor>
    void _visit_elements_indexed(Visitor&& visitor)
    {
        this->VariadicVectorContainer<T>::_for_each_element_indexed(visitor);
        if constexpr (sizeof...(Ts) > 0)
        {
            _visit_elements_indexed<Ts...>(std::forward<Visitor>(visitor));
        }
    }

    template<typename T, typename... Ts>
    size_t _size_helper() const
    {
        size_t sizeT = this->VariadicVectorContainer<T>::_size();
        size_t sizeTs = 0;
        if constexpr (sizeof...(Ts) > 0)
        {
            sizeTs = _size_helper<Ts...>();
        }
        
        return sizeT + sizeTs;
    }

    template<typename T, typename... Ts>
    void _clear_helper()
    {
        clear<T>();
        if constexpr (sizeof...(Ts) > 0)
        {
            _clear_helper<Ts...>();
        }
    }
};
