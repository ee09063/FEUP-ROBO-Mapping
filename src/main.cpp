#include "../include/mapping.h"

int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_mapping", ros::init_options::AnonymousName);
  jpmsc_mapping::Mapping obj(argc, argv);
  ros::spin();
  return 0;
}
