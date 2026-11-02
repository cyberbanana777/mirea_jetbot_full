#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include "serial_bridge_cpp/frame_parser.hpp"
int main(){
 std::vector<double> values;
 assert(serial_bridge_cpp::parse_fields("$301;1;2;3;4;5;6;7;#",values));
 assert(values.size()==8 && values[0]==301);
 assert(!serial_bridge_cpp::parse_fields("$301;nan;#",values));
 serial_bridge_cpp::FrameParser p;
 int count=0;
 std::string data="$301;0;0;0;0;0;0;0;#\r\n$202;1;2;3;4;5;6;#";
 p.feed(data.data(),data.size(),[&](const std::string&){++count;});
 assert(count==2);
}
