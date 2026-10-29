#include <iostream>
#include <iomanip>
#include <cmath>
#include "image_admission.h"
int main() {
 for (double target_hz : {7.5, 10.0}) {
  const double base=1793220000.0;
  const double min_interval=1.0/target_hz;
  double last=-1.0;
  int accept=0, too_soon=0;
  for (int i=0;i<240;i++) {
   double ts=base+(double(i)/30.0);
   auto decision=imageAdmissionDecision(ts,last,min_interval);
   if(decision==ImageAdmissionDecision::ACCEPT){last=ts; accept++;}
   if(decision==ImageAdmissionDecision::TOO_SOON)too_soon++;
  }
  std::cout<<std::fixed<<std::setprecision(3)<<"camera_input_hz=30 target_hz="<<target_hz
           <<" eight_second_frames=240 accepted="<<accept<<" too_soon="<<too_soon
           <<" actual_hz="<<(accept/8.0)<<"\n";
 }
}
