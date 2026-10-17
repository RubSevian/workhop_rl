#include "sdk_mode_session.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
// Synthetic child only: tests never initialize SDK or communicate with a robot.
int main(){
 using namespace sim2real;
 const auto fd_count=[](){return std::distance(std::filesystem::directory_iterator("/proc/self/fd"),std::filesystem::directory_iterator{});};
 const auto initial_fds=fd_count();
 const auto dir=std::filesystem::temp_directory_path()/("r3-session-"+std::to_string(getpid()));
 std::filesystem::create_directory(dir);const auto exe=dir/"fake";
 const auto starts=dir/"starts";
 auto script=[&](const std::string& body){std::ofstream(exe)<<"#!/usr/bin/python3\nimport sys,time,os\n"<<body;
  std::filesystem::permissions(exe,std::filesystem::perms::owner_all);};
 auto response=[](const std::string& state){return " print('R3_MODE_V1', ident, '"+state+"', start, time.monotonic_ns(), 1, 'fake', flush=True)\n";};
 script("open('"+starts.string()+"','a').write('start\\n')\n"
        "assert sys.argv[1:] == ['--interface','iface;$(never_execute)','--session']\n"
        "print('SDK startup diagnostic',flush=True)\n"
        "for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n"
        " assert command in ('STATUS','RELEASE')\n"+response("SPORT_RELEASED"));
 {
  ModeSessionProcess session(exe,"iface;$(never_execute)");
  for(int i=0;i<8;++i){auto r=session.Request(false);if(!r.ok)std::cerr<<r.detail<<std::endl;assert(r.ok&&r.state=="SPORT_RELEASED"&&r.completed_ns>=r.started_ns);}
  assert(session.Request(true).ok);
  std::ifstream file(starts);std::string line;int count=0;while(std::getline(file,line))++count;
  assert(count==1); // All status/release RPCs use the same child.
 }
 script("assert sys.argv[1:] == ['--interface','test','--session','--session-config','/offline/config with spaces.yaml']\n"
        "for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n"+response("SPORT_RELEASED"));
 {ModeSessionProcess session(exe,"test","/offline/config with spaces.yaml");assert(session.Request(false).ok);}
 script("for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n"+response("SPORT_ACTIVE"));
 {ModeSessionProcess session(exe,"test");assert(session.Request(false).ok);assert(!session.Request(true).ok);}
 script("for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n"
        " print('R3_MODE_V1',int(ident)+1,'SPORT_RELEASED',start,time.monotonic_ns(),1,flush=True)\n");
 {ModeSessionProcess session(exe,"test");assert(!session.Request(false).ok);}
 script("for line in sys.stdin:\n ident,command=line.split()\n"
        " print('R3_MODE_V1',ident,'SPORT_RELEASED',1,2,1,flush=True)\n");
 {ModeSessionProcess session(exe,"test");assert(!session.Request(false).ok);}
 script("for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n"
        " print('R3_MODE_V1',ident,'SPORT_ERROR',start,time.monotonic_ns(),0,'RPC timeout',flush=True)\n");
 {ModeSessionProcess session(exe,"test");auto r=session.Request(false);assert(!r.ok&&r.detail.find("RPC timeout")!=std::string::npos);}
 script("for line in sys.stdin: time.sleep(3)\n");
 {ModeSessionProcess session(exe,"test");const auto start=std::chrono::steady_clock::now();assert(!session.Request(false,50).ok);
  assert(std::chrono::steady_clock::now()-start<std::chrono::seconds(1));
  // After timeout a new process may only perform the explicitly requested STATUS.
  script("for line in sys.stdin:\n ident,command=line.split(); start=time.monotonic_ns()\n assert command=='STATUS'\n"+response("SPORT_RELEASED"));
  assert(session.Request(false).ok);}
 script("raise SystemExit(1)\n");
 {ModeSessionProcess session(exe,"test");assert(!session.Request(false).ok);}
 {ModeSessionProcess session("/missing/helper","test");assert(!session.Request(false).ok);}
 assert(fd_count()==initial_fds);
 int child_status=0;assert(waitpid(-1,&child_status,WNOHANG)==-1&&errno==ECHILD);
 std::filesystem::remove_all(dir);std::cout<<"PASS persistent SDK protocol: reuse, ownership states, timestamps, timeout, restart, EOF\n";
}
