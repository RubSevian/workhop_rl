#include "sdk_mode_process.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
// Fake executables only: no Unitree SDK, DDS or robot connection.
int main(){
 using namespace sim2real;
 const auto dir=std::filesystem::temp_directory_path()/("r3-helper-"+std::to_string(getpid()));
 std::filesystem::create_directory(dir);const auto exe=dir/"fake_helper";
 auto script=[&](const std::string& body){std::ofstream(exe)<<"#!/usr/bin/python3\n"<<body;
  std::filesystem::permissions(exe,std::filesystem::perms::owner_all);};
 script("print('SPORT_ACTIVE')\n");assert(RunModeProcess(exe,"test-interface",false).ok);
 assert(!RunModeProcess(exe,"test-interface",true).ok);
 script("import sys\nassert sys.argv[1:] == ['--interface', 'iface;$(never_execute)', '--release-sport-mode', '--machine-readable']\nprint('SPORT_RELEASED')\n");
 assert(RunModeProcess(exe,"iface;$(never_execute)",true).ok); // argument, never a shell
 script("print('SPORT_RELEASED')\nraise SystemExit(1)\n");assert(!RunModeProcess(exe,"test",true).ok);
 script("print('SPORT_ACTIVE')\nprint('SPORT_RELEASED')\n");assert(!RunModeProcess(exe,"test",false).ok);
 script("import time\ntime.sleep(3)\n");auto start=std::chrono::steady_clock::now();
 assert(!RunModeProcess(exe,"test",true,50).ok);
 assert(std::chrono::steady_clock::now()-start<std::chrono::seconds(1));
 assert(!RunModeProcess("/missing/helper","test",true).ok);
 assert(!RunModeProcess(exe,"",true).ok);
 std::filesystem::remove_all(dir);std::cout<<"PASS fake SDK process: semantic result, exit code, no shell, bounded timeout\n";
}
