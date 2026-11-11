#include "receiver_schedstat.hpp"
#include <cassert>
int main(){using sim2real::ParseRunnableWaitNs;
assert(ParseRunnableWaitNs("123 456 7\n")==456);
assert(ParseRunnableWaitNs("123 0 7\n")==0);
assert(ParseRunnableWaitNs("123 -1 7\n")==-1);
assert(ParseRunnableWaitNs("123 18446744073709551615 7")==-1);
assert(ParseRunnableWaitNs("bad")==-1);
}
