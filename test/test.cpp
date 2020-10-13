#include <ArduinoUnitTests.h>
#include <LimitedPID.h>
#include <PID.h>

using namespace arc;

unittest(PID) {
  double target = 30.0;
  unsigned long t = 0;
  PID<double> pid(1.0, 0.5, 0.9);
  pid.setInput(0, t);
  double g = 9.8;
  double ac = 0;
  double ds = 0;
  int inc = 10;
  pid.setTarget(target);
  for (int i = 0; i < 1000; i++) {
    t += inc;
    pid.setInput(ds, t);
    double result = pid.getOutput();
    ac += result;
    ds += (ac - g) * ((double)inc / 1000.0);
    // printf("r=%f,ac=%f,ds=%f,t=%d\n",result,ac,ds,t);
  }
  assertEqual(round(ds), target);
}

unittest(LimitedPID) {
  double target = 30.0;
  unsigned long t = 0;
  PID<double> pid(1.0, 0.5, 0.9);
  LimitedPID<double> rpid(pid, std::make_pair(0, 100));
  double g = 9.8;
  double ac = 0;
  double ds = 0;
  int inc = 10;
  rpid.setTarget(target);
  for (int i = 0; i < 1000; i++) {
    t += inc;
    rpid.setInput(ds, t);
    ac = rpid.getOutput(ac);
    ds += (ac - g) * ((double)inc / 1000.0);
    // printf("ac=%f,ds=%f,t=%d\n",ac,ds,t);
  }
  assertEqual(round(ds), target);
}

unittest_main()