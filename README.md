# xl320
Dynamixel XL-320

## Example

```
#include "motors.h"

DM2 m(p13, p14); // Tx, Rx, Baud=1000000

...

int ec = 0; // Error code
ec = m.SetGoalPosition(1, encode_theta(d.Theta1()));
if (ec != 0) {
	// Handle error
}

```