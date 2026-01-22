
#pragma once

#include <vector>
#include <iostream>

/** A consistent reference to an element in a vector.
 * Things are stored in vectors which can dynamically change size. If a vector has to allocate more memory, any pointers or references
 * to its contents are invalidated. This becomes a problem for things that are dynamically added and removed (like constraints).
 * 
 * By storing a pointer to the container and its index, we can ensure that even if the vector changes sizes, we still have a valid
 * reference to the element.
 * 
 * Note: if the template parameter T is a const type, then the underlying vector cannot be modified by the VectorHandle.
 * 
 * There is a performance hit by accessing the element through the vector rather than with direct pointer, and
 * this may become slightly significant for things accessed this way in the inner loop of the simulation.
 * 
 * But, for now I think the flexibility (and not having to worry about pre-allocating enough space in the constraint vectors) outweighs the slight cost.
 * 
 * 
 */
template <typename T>
class VectorHandle
{
public:
    using element_type = T;
    using vector_type = std::vector<T>;
    
    VectorHandle(vector_type* vec, int index)
        : _vec(vec), _index(index)
    {
    }

    VectorHandle()
        : _vec(0), _index(0)
    {

    }

    const element_type* operator->() const
    {
        // return _ptr;
        return &(*_vec).at(_index);
    }

    element_type* operator->()
    {
        // return _ptr;
        return &(*_vec).at(_index);
    }

    const element_type& get() const
    {
        return (*_vec).at(_index);
    }

    element_type& get()
    {
        return (*_vec).at(_index);
    }

    private:
    vector_type* _vec;
    int _index;
};



////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////



template <typename T>
class ConstVectorHandle
{
public:
    using element_type = T;
    using vector_type = std::vector<T>;    // this makes it work for const types too
    
    ConstVectorHandle(const vector_type* vec, int index)
        : _vec(vec), _index(index)
    {
    }

    ConstVectorHandle()
        : _vec(0), _index(0)
    {

    }

    const element_type* operator->() const
    {
        // return _ptr;
        return &(*_vec).at(_index);
    }

    const element_type& get() const
    {
        return (*_vec).at(_index);
    }

    private:
    const vector_type* _vec;
    int _index;
};