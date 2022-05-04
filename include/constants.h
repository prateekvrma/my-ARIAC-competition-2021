#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
  namespace PriorityWeight {
      namespace Ratio {
        const int HIGH_PRIORITY = 10; 
      }

      namespace Penalty {
        const int NO_PART = -3;   
        const int MOVE_FAILS = -1; 
        const int SHIPMENT_POSTPONE = -1; 
      }

      namespace Reward {
        const int SHIPMENT_READY = 1; 
      }

      namespace Level {
        const int EMERGENCY = 10; 
        const int HIGH = 5; 
        const int LOW = 2; 
      }
     namespace Priority{
	const int order = 100;
	const int pump = 1;
	const int as1 = 10;
	const int as2 = 20;
	const int as3 = 30;
	const int as4 = 40;
     }	
  }
}

#endif 
