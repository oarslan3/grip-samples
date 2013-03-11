// ====================================================================
//      Original implementation of Fibonacci Heap is done by Max Winkler.
//      Then, its template version is implemented by Oktay Arslan.
// ====================================================================

#include "FibonacciHeapTemplate.h"

#include <vector>
#include <cmath>

//
CostKey::CostKey()
{
    key1 = 0;
    key2 = 0;
}

bool
operator==(const CostKey& x, const CostKey& y)
{
    return (x.key1 == y.key1) && (x.key2 == y.key2);
}

bool
operator<=(const CostKey& x, const CostKey& y)
{
    return (x.key1 < y.key1) || (x.key1 == y.key1 && x.key2 <= y.key2);
}

bool
operator>=(const CostKey& x, const CostKey& y)
{
    return (x.key1 > y.key1) || (x.key1 == y.key1 && x.key2 >= y.key2);
}

bool
operator<(const CostKey& x, const CostKey& y)
{
    return (x.key1 < y.key1) || (x.key1 == y.key1 && x.key2 < y.key2);
}

bool
operator>(const CostKey& x, const CostKey& y)
{
    return (x.key1 > y.key1) || (x.key1 == y.key1 && x.key2 > y.key2);
}
