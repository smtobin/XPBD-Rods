#pragma once

#include "common/common.hpp"
#include <numeric>

struct Algorithm
{
/** Radix sort on uint64 keys.
 * @param unsorted the list of unsorted keys
 * @param sorted_order (output) the indices of the unsorted array, arranged in the sorted order
 * @param size the size of the vector (the unsorted vector may have extra padding)
 */
static void radixSort(const std::vector<uint64_t>& unsorted, std::vector<unsigned>& sorted_order, unsigned size)
{
    sorted_order.resize(size);
    std::iota(sorted_order.begin(), sorted_order.end(), 0);

    std::vector<unsigned> temp(size);

    constexpr int BITS = 8;
    constexpr int BUCKETS = 1 << BITS;
    constexpr int MASK = BUCKETS - 1;

    for (int shift = 0; shift < 64; shift += BITS)
    {
        size_t count[BUCKETS] = {};

        // count
        for (unsigned idx : sorted_order)
            ++count[(unsorted[idx] >> shift) & MASK];
        
        // prefix sums
        size_t sum = 0;
        for (int i = 0; i < BUCKETS; i++)
        {
            size_t c = count[i];
            count[i] = sum;
            sum += c;
        }

        // scatter
        for (unsigned idx : sorted_order)
            temp[count[(unsorted[idx] >> shift) & MASK]++] = idx;

        sorted_order.swap(temp);
    }
}

};