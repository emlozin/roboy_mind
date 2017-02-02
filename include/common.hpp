/** @file common.hpp
 *  @brief File storing helper functions and structures declarations
 */

 // Standard libraries
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string.h>

#include <stdlib.h>
#include <string>
#include <cmath>
#include <time.h>
#include <vector>
#include <algorithm>
#include <map>

#include <mutex>
#include <thread>
#include <pthread.h>

using namespace std;

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

/* **********************
* STRING/INT CONVERSION
* **********************/

/** Function converting integer to string
*  @param a   number that needs to be converted
*  @return    resulting string
*/
string IntToStr(int a){
    stringstream ss;
    ss << a;
    return ss.str();
}

/** Function converting float to string
 *  @param a   number that needs to be converted
 *  @return    resulting string
 */
 string FloatToStr(float a){
     stringstream ss;
     ss << a;
     return ss.str();
 }

/** Function converting string to float
 *  @param a   string that needs to be converted
 *  @return    resulting float value
 */
 float StrToFloat(string a){
     float result;
     result = stof(a);
     result = round(int(result * 100))/100;
     return result;
 }

/** Function converting string to integer
 *  @param a   string that needs to be converted
 *  @return    resulting integer value
 */
 int StrToInt(string a){
     float result;
     result = stoi(a);
     return result;
 }
