/*
 * functions.h
 *
 *  Created on: 05.12.2018
 *      Author: mct
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

const uint16_t SINUS[] = {
		512,520,529,538,547,556,565,574,
		583,592,600,609,618,627,635,644,
		652,661,670,678,686,695,703,711,
		720,728,736,744,752,759,767,775,
		783,790,798,805,812,819,826,833,
		840,847,854,860,867,873,879,886,
		892,898,903,909,915,920,925,930,
		936,940,945,950,954,959,963,967,
		971,975,979,982,986,989,992,995,
		998,1001,1003,1006,1008,1010,1012,1014,
		1015,1017,1018,1019,1020,1021,1022,1022,
		1023,1023,1023,1023,1023,1022,1022,1021,
		1020,1019,1018,1017,1015,1014,1012,1010,
		1008,1006,1003,1001,998,995,992,989,
		986,982,979,975,971,967,963,959,
		954,950,945,940,936,930,925,920,
		915,909,903,898,892,886,879,873,
		867,860,854,847,840,833,826,819,
		812,805,798,790,783,775,767,759,
		752,744,736,728,720,711,703,695,
		686,678,670,661,652,644,635,627,
		618,609,600,592,583,574,565,556,
		547,538,529,520,512,503,494,485,
		476,467,458,449,440,431,423,414,
		405,396,388,379,371,362,353,345,
		337,328,320,312,303,295,287,279,
		271,264,256,248,240,233,225,218,
		211,204,197,190,183,176,169,163,
		156,150,144,137,131,125,120,114,
		108,103,98,93,87,83,78,73,
		69,64,60,56,52,48,44,41,
		37,34,31,28,25,22,20,17,
		15,13,11,9,8,6,5,4,
		3,2,1,1,0,0,0,0,
		0,1,1,2,3,4,5,6,
		8,9,11,13,15,17,20,22,
		25,28,31,34,37,41,44,48,
		52,56,60,64,69,73,78,83,
		87,93,98,103,108,114,120,125,
		131,137,144,150,156,163,169,176,
		183,190,197,204,211,218,225,233,
		240,248,256,264,271,279,287,295,
		303,312,320,328,337,345,353,362,
		371,379,388,396,405,414,423,431,
		440,449,458,467,476,485,494,503};

const uint16_t DREIECK[] = {
		6,11,17,23,28,34,40,45,
		51,57,63,68,74,80,85,91,
		97,102,108,114,119,125,131,136,
		142,148,153,159,165,171,176,182,
		188,193,199,205,210,216,222,227,
		233,239,244,250,256,261,267,273,
		278,284,290,296,301,307,313,318,
		324,330,335,341,347,352,358,364,
		369,375,381,386,392,398,404,409,
		415,421,426,432,438,443,449,455,
		460,466,472,477,483,489,494,500,
		506,512,517,523,529,534,540,546,
		551,557,563,568,574,580,585,591,
		597,602,608,614,619,625,631,637,
		642,648,654,659,665,671,676,682,
		688,693,699,705,710,716,722,727,
		733,739,745,750,756,762,767,773,
		779,784,790,796,801,807,813,818,
		824,830,835,841,847,852,858,864,
		870,875,881,887,892,898,904,909,
		915,921,926,932,938,943,949,955,
		960,966,972,978,983,989,995,1000,
		1006,1012,1017,1023,1017,1012,1006,1000,
		995,989,983,978,972,966,960,955,
		949,943,938,932,926,921,915,909,
		904,898,892,887,881,875,870,864,
		858,852,847,841,835,830,824,818,
		813,807,801,796,790,784,779,773,
		767,762,756,750,745,739,733,727,
		722,716,710,705,699,693,688,682,
		676,671,665,659,654,648,642,637,
		631,625,619,614,608,602,597,591,
		585,580,574,568,563,557,551,546,
		540,534,529,523,517,512,506,500,
		494,489,483,477,472,466,460,455,
		449,443,438,432,426,421,415,409,
		404,398,392,386,381,375,369,364,
		358,352,347,341,335,330,324,318,
		313,307,301,296,290,284,278,273,
		267,261,256,250,244,239,233,227,
		222,216,210,205,199,193,188,182,
		176,171,165,159,153,148,142,136,
		131,125,119,114,108,102,97,91,
		85,80,74,68,63,57,51,45,
		40,34,28,23,17,11,6,0};

const uint16_t SAEGEZAHN[] = {
		3,6,9,11,14,17,20,23,
		26,28,31,34,37,40,43,45,
		48,51,54,57,60,63,65,68,
		71,74,77,80,82,85,88,91,
		94,97,99,102,105,108,111,114,
		117,119,122,125,128,131,134,136,
		139,142,145,148,151,153,156,159,
		162,165,168,171,173,176,179,182,
		185,188,190,193,196,199,202,205,
		207,210,213,216,219,222,224,227,
		230,233,236,239,242,244,247,250,
		253,256,259,261,264,267,270,273,
		276,278,281,284,287,290,293,296,
		298,301,304,307,310,313,315,318,
		321,324,327,330,332,335,338,341,
		344,347,350,352,355,358,361,364,
		367,369,372,375,378,381,384,386,
		389,392,395,398,401,404,406,409,
		412,415,418,421,423,426,429,432,
		435,438,440,443,446,449,452,455,
		458,460,463,466,469,472,475,477,
		480,483,486,489,492,494,497,500,
		503,506,509,511,514,517,520,523,
		526,529,531,534,537,540,543,546,
		548,551,554,557,560,563,565,568,
		571,574,577,580,583,585,588,591,
		594,597,600,602,605,608,611,614,
		617,619,622,625,628,631,634,637,
		639,642,645,648,651,654,656,659,
		662,665,668,671,673,676,679,682,
		685,688,691,693,696,699,702,705,
		708,710,713,716,719,722,725,727,
		730,733,736,739,742,745,747,750,
		753,756,759,762,764,767,770,773,
		776,779,781,784,787,790,793,796,
		799,801,804,807,810,813,816,818,
		821,824,827,830,833,835,838,841,
		844,847,850,853,855,858,861,864,
		867,870,872,875,878,881,884,887,
		889,892,895,898,901,904,906,909,
		912,915,918,921,924,926,929,932,
		935,938,941,943,946,949,952,955,
		958,960,963,966,969,972,975,978,
		980,983,986,989,992,995,997,1000,
		1003,1006,1009,1012,1014,1017,1020,1023,0};

#endif /* FUNCTIONS_H_ */