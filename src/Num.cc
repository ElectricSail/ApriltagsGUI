#include <iostream>
#include "Num.h"

namespace AprilTags{
	void Num::set(int m, int d, int y) {
	month = m; day = d; year = y;
	}
	void Num::print() {
	std::cout << month << " / " << day
	<< " / " << year << std::endl;
	}
}


