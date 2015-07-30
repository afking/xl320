# xl320
Dynamixel XL-320

## Example

```
#include "motors.h"

DM2 m(p13, p14); // Tx, Rx, Baud=1000000

...

int encode_theta(const double &t) {
	int md = (int) (t*195.5696 + 512.5); // Int cast (t)/(150/180*pi)*512
	if (md > 1024) {
		md = 1024;
	}
	if (md < 0) {
		md = 0;
	}
	return md;
}

... {
	int ec = 0; // Error code
	ec = m.SetGoalPosition(1, encode_theta(angle_to_encode));
	if (ec != 0) {
		// Handle error
	}
}

```