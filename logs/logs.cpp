#include "logs.hpp"

using namespace std;

string getLColor(int color, bool bright, bool background)
{
	return string("\033[") + string(bright ? "1" : "21") + string(";") + to_string((int)color + (background ? 40 : 30)) + string("m");
}
