
#include <dlfcn.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include "Common.h"
int main(int argc,char**argv) {
  void* lib=dlopen(argv[1],RTLD_LAZY|RTLD_LOCAL);
  if(!lib){std::cerr<<dlerror();return 1;}
  using Convert=std::vector<double>(*)(LINKER_HAND,HAND_TYPE,const std::vector<uint8_t>&);
  auto convert=reinterpret_cast<Convert>(dlsym(lib,"_Z12range_to_arc11LINKER_HAND9HAND_TYPERKSt6vectorIhSaIhEE"));
  if(!convert){std::cerr<<dlerror();return 2;}
  // Only pure conversion; no hand object, callbacks, bus, socket or CAN library.
  std::cout<<std::setprecision(17)<<"{\"model\":\"L20_10=G20\",\"side\":\"left\",\"raw_to_rad\":[";
  for(int u=0;u<256;++u){
    auto q=convert(L20_10,LEFT,std::vector<uint8_t>(16,u));
    if(q.size()!=16){std::cerr<<"unexpected joint count "<<q.size();return 3;}
    if(u)std::cout<<",";std::cout<<"[";
    for(int j=0;j<16;++j){if(j)std::cout<<",";std::cout<<q[j];}
    std::cout<<"]";
  }
  std::cout<<"]}\n";
}
