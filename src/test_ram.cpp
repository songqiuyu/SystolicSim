#include "base/base.h"
#include "base/config.h"
#include "memory_system/memory_system.h"
int main() { auto cfg = Ramulator::Config::parse_config_file("../ramulator2/example_config.yaml", {}); auto mem = Ramulator::Factory::create_memory_system(cfg); return 0; }
