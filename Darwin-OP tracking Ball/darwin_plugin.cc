// Author : Bharadwaj-Ramesh
// Date   : 8th March 2013
// Email  : br375@drexel.edu
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <fstream>

// This program makes the robot walk towards the ball.
// rename the plugin to darwin_plugin.cc and then build it 
      float hipYr_loc [] =
{0,0,0,0,0,0,0,0,0.00026835,0.002098,0.006814,0.015305,0.027888,0.044263,0.063553,0.084443,0.10538,0.12479,0.14134,0.15414,0.16284,0.16774,0.16969,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.17,0.16973,0.1679,0.16319,0.1547,0.14211,0.12574,0.10645,0.085557,0.064624,0.045214,0.02866,0.015864,0.0071609,0.002264,0.00031266,3.9249e-08,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      float hipYl_loc [] = 
{0,-0,-0,-0,-0,-0,-0,-0,-0.00026835,-0.002098,-0.006814,-0.015305,-0.027888,-0.044263,-0.063553,-0.084443,-0.10538,-0.12479,-0.14134,-0.15414,-0.16284,-0.16774,-0.16969,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.17,-0.16973,-0.1679,-0.16319,-0.1547,-0.14211,-0.12574,-0.10645,-0.085557,-0.064624,-0.045214,-0.02866,-0.015864,-0.0071609,-0.002264,-0.00031266,-3.9249e-08,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0,-0};
//      float xp2[2] = {0,1};
      float xp [192] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191};
      float footl_loc [] = {0,-0.52886,-0.53511,-0.53511,-0.53511,-0.53511,-0.53511,-0.53511,-0.53515,-0.5353,-0.53553,-0.53585,-0.53626,-0.53674,-0.5373,-0.53793,-0.53862,-0.53938,-0.5402,-0.54107,-0.54199,-0.54296,-0.54396,-0.54501,-0.54609,-0.5472,-0.54833,-0.54949,-0.55067,-0.55185,-0.55305,-0.55426,-0.55546,-0.55667,-0.55787,-0.55906,-0.56023,-0.5614,-0.56254,-0.56365,-0.56474,-0.5658,-0.56683,-0.56782,-0.56876,-0.56966,-0.57052,-0.57132,-0.57206,-0.57275,-0.57286,-0.57497,-0.57702,-0.57899,-0.58091,-0.58276,-0.61682,-0.64912,-0.67956,-0.708,-0.7343,-0.75832,-0.7799,-0.79891,-0.81523,-0.82876,-0.83943,-0.8472,-0.8521,-0.85415,-0.85346,-0.85016,-0.84442,-0.83643,-0.82642,-0.81462,-0.80127,-0.78658,-0.77075,-0.75396,-0.73635,-0.718,-0.69898,-0.6793,-0.65894,-0.63784,-0.61591,-0.59304,-0.56909,-0.54392,-0.51735,-0.48922,-0.48676,-0.48881,-0.4909,-0.49306,-0.49526,-0.49477,-0.49719,-0.49952,-0.50179,-0.50398,-0.5061,-0.50816,-0.51016,-0.5121,-0.51398,-0.51582,-0.51761,-0.51935,-0.52106,-0.52272,-0.52436,-0.52595,-0.52753,-0.52907,-0.5306,-0.5321,-0.53359,-0.53506,-0.53652,-0.53797,-0.53942,-0.54087,-0.54232,-0.54377,-0.54522,-0.54668,-0.54816,-0.54965,-0.55115,-0.55267,-0.55422,-0.55579,-0.55738,-0.559,-0.56066,-0.56234,-0.56407,-0.56583,-0.56762,-0.56947,-0.57135,-0.57329,-0.57286,-0.57496,-0.57695,-0.57885,-0.58065,-0.58236,-0.61625,-0.6484,-0.67876,-0.70727,-0.73386,-0.75843,-0.78089,-0.80116,-0.81915,-0.83479,-0.84802,-0.85882,-0.86717,-0.87307,-0.87655,-0.87768,-0.87652,-0.87316,-0.86772,-0.86031,-0.85105,-0.84005,-0.82743,-0.81329,-0.79771,-0.78075,-0.76248,-0.74292,-0.72208,-0.69996,-0.67655,-0.6518,-0.62568,-0.59811,-0.56902,-0.53833,-0.53401,-0.53429,-0.53457,-0.53487,-0.53517,0};
      float footr_loc [] = {0,0.52886,0.53511,0.53511,0.53511,0.53511,0.53511,0.53511,0.56636,0.59597,0.62388,0.65005,0.67438,0.69681,0.71726,0.73564,0.75188,0.76595,0.77778,0.78738,0.79473,0.79986,0.80282,0.80366,0.80248,0.79936,0.79441,0.78773,0.77944,0.76963,0.75839,0.74581,0.73193,0.7168,0.70044,0.68285,0.66403,0.64394,0.62254,0.59977,0.57557,0.54985,0.52253,0.4935,0.4901,0.49113,0.4921,0.49301,0.49386,0.49465,0.49477,0.49719,0.49952,0.50179,0.50398,0.5061,0.50816,0.51016,0.5121,0.51398,0.51582,0.51761,0.51935,0.52106,0.52272,0.52436,0.52595,0.52753,0.52907,0.5306,0.5321,0.53359,0.53506,0.53652,0.53797,0.53942,0.54087,0.54232,0.54377,0.54522,0.54668,0.54816,0.54965,0.55115,0.55267,0.55422,0.55579,0.55738,0.559,0.56066,0.56234,0.56407,0.56583,0.56762,0.56947,0.57135,0.57329,0.57286,0.57497,0.57702,0.57899,0.58091,0.58276,0.61682,0.64912,0.67956,0.708,0.7343,0.75832,0.7799,0.79891,0.81523,0.82876,0.83943,0.8472,0.8521,0.85415,0.85346,0.85016,0.84442,0.83643,0.82642,0.81462,0.80127,0.78658,0.77075,0.75396,0.73635,0.718,0.69898,0.6793,0.65894,0.63784,0.61591,0.59304,0.56909,0.54392,0.51735,0.48922,0.48676,0.48881,0.4909,0.49306,0.49526,0.49477,0.49717,0.49945,0.50162,0.50369,0.50565,0.50751,0.50927,0.51094,0.51252,0.51401,0.51542,0.51676,0.51801,0.51919,0.5203,0.52134,0.52232,0.52324,0.52409,0.5249,0.52564,0.52634,0.527,0.5276,0.52817,0.52869,0.52918,0.52964,0.53007,0.53046,0.53084,0.53119,0.53152,0.53183,0.53212,0.53241,0.53269,0.53295,0.53322,0.53348,0.53375,0.53401,0.53429,0.53457,0.53487,0.53517,0};
      float anklel_loc [] = {0,0,0.01538,0.030685,0.045839,0.06077,0.075406,0.089678,0.10352,0.11687,0.12966,0.14185,0.15338,0.16419,0.17426,0.18354,0.192,0.1996,0.20632,0.21214,0.21703,0.22098,0.22398,0.22601,0.22708,0.22718,0.2263,0.22446,0.22165,0.21788,0.21318,0.20754,0.201,0.19357,0.18528,0.17616,0.16625,0.15557,0.14418,0.13212,0.11944,0.1062,0.092453,0.078261,0.063691,0.048813,0.033697,0.018416,0.003045,-0.012341,-0.027666,-0.042857,-0.057838,-0.072538,-0.086887,-0.10082,-0.11427,-0.12718,-0.13949,-0.15115,-0.16211,-0.17233,-0.18177,-0.19039,-0.19817,-0.20506,-0.21106,-0.21613,-0.22027,-0.22346,-0.22569,-0.22695,-0.22724,-0.22655,-0.2249,-0.22228,-0.2187,-0.21418,-0.20873,-0.20237,-0.19511,-0.18699,-0.17803,-0.16827,-0.15774,-0.14649,-0.13456,-0.122,-0.10886,-0.09521,-0.0811,-0.0666,-0.051777,-0.036703,-0.021448,-0.0060893,0.0092996,0.024643,0.039866,0.054894,0.069655,0.084079,0.098099,0.11165,0.12467,0.1371,0.1489,0.16,0.17037,0.17997,0.18875,0.1967,0.20377,0.20994,0.2152,0.21953,0.2229,0.22532,0.22677,0.22726,0.22676,0.2253,0.22287,0.21949,0.21515,0.20988,0.2037,0.19662,0.18866,0.17987,0.17026,0.15988,0.14877,0.13697,0.12453,0.1115,0.097949,0.083925,0.069497,0.054732,0.039701,0.024477,0.0091326,-0.0062565,-0.021615,-0.036867,-0.05194,-0.06676,-0.081256,-0.095361,-0.10901,-0.12214,-0.13469,-0.14662,-0.15786,-0.16838,-0.17813,-0.18708,-0.19519,-0.20244,-0.20879,-0.21424,-0.21875,-0.22231,-0.22492,-0.22656,-0.22724,-0.22694,-0.22567,-0.22343,-0.22023,-0.21608,-0.211,-0.20499,-0.19809,-0.1903,-0.18167,-0.17223,-0.162,-0.15103,-0.13936,-0.12704,-0.11413,-0.10067,-0.086733,-0.07238,-0.057676,-0.042693,-0.027501,-0.012174,0};
      float ankler_loc [] = {0,0,0.01538,0.030685,0.045839,0.06077,0.075406,0.089678,0.10352,0.11687,0.12966,0.14185,0.15338,0.16419,0.17426,0.18354,0.192,0.1996,0.20632,0.21214,0.21703,0.22098,0.22398,0.22601,0.22708,0.22718,0.2263,0.22446,0.22165,0.21788,0.21318,0.20754,0.201,0.19357,0.18528,0.17616,0.16625,0.15557,0.14418,0.13212,0.11944,0.1062,0.092453,0.078261,0.063691,0.048813,0.033697,0.018416,0.003045,-0.012341,-0.027666,-0.042857,-0.057838,-0.072538,-0.086887,-0.10082,-0.11427,-0.12718,-0.13949,-0.15115,-0.16211,-0.17233,-0.18177,-0.19039,-0.19817,-0.20506,-0.21106,-0.21613,-0.22027,-0.22346,-0.22569,-0.22695,-0.22724,-0.22655,-0.2249,-0.22228,-0.2187,-0.21418,-0.20873,-0.20237,-0.19511,-0.18699,-0.17803,-0.16827,-0.15774,-0.14649,-0.13456,-0.122,-0.10886,-0.09521,-0.0811,-0.0666,-0.051777,-0.036703,-0.021448,-0.0060893,0.0092996,0.024643,0.039866,0.054894,0.069655,0.084079,0.098099,0.11165,0.12467,0.1371,0.1489,0.16,0.17037,0.17997,0.18875,0.1967,0.20377,0.20994,0.2152,0.21953,0.2229,0.22532,0.22677,0.22726,0.22676,0.2253,0.22287,0.21949,0.21515,0.20988,0.2037,0.19662,0.18866,0.17987,0.17026,0.15988,0.14877,0.13697,0.12453,0.1115,0.097949,0.083925,0.069497,0.054732,0.039701,0.024477,0.0091326,-0.0062565,-0.021615,-0.036867,-0.05194,-0.06676,-0.081256,-0.095361,-0.10901,-0.12214,-0.13469,-0.14662,-0.15786,-0.16838,-0.17813,-0.18708,-0.19519,-0.20244,-0.20879,-0.21424,-0.21875,-0.22231,-0.22492,-0.22656,-0.22724,-0.22694,-0.22567,-0.22343,-0.22023,-0.21608,-0.211,-0.20499,-0.19809,-0.1903,-0.18167,-0.17223,-0.162,-0.15103,-0.13936,-0.12704,-0.11413,-0.10067,-0.086733,-0.07238,-0.057676,-0.042693,-0.027501,-0.012174,0};
      float kneel_loc [] = {0,-1.0577,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0701,-1.0701,-1.0701,-1.0701,-1.07,-1.07,-1.07,-1.0699,-1.0699,-1.0698,-1.0697,-1.0696,-1.0696,-1.0695,-1.0694,-1.0693,-1.0692,-1.0691,-1.069,-1.0689,-1.0688,-1.0686,-1.0685,-1.0684,-1.0683,-1.0682,-1.0681,-1.068,-1.0678,-1.0677,-1.0676,-1.0676,-1.0673,-1.067,-1.0667,-1.0664,-1.066,-1.1283,-1.1871,-1.2428,-1.2951,-1.3443,-1.3902,-1.4328,-1.4721,-1.5081,-1.5406,-1.5696,-1.595,-1.6167,-1.6347,-1.6488,-1.659,-1.6653,-1.6676,-1.666,-1.6603,-1.6507,-1.6372,-1.6198,-1.5986,-1.5738,-1.5453,-1.5133,-1.4779,-1.439,-1.3969,-1.3514,-1.3027,-1.2508,-1.1957,-1.1372,-1.0754,-1.0666,-1.0668,-1.0671,-1.0674,-1.0677,-1.0676,-1.0679,-1.0682,-1.0684,-1.0687,-1.0689,-1.069,-1.0692,-1.0693,-1.0695,-1.0696,-1.0697,-1.0698,-1.0699,-1.07,-1.07,-1.0701,-1.0701,-1.0701,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,-1.0701,-1.0701,-1.07,-1.07,-1.0699,-1.0698,-1.0698,-1.0697,-1.0696,-1.0695,-1.0693,-1.0692,-1.0691,-1.0689,-1.0687,-1.0685,-1.0683,-1.0681,-1.0678,-1.0676,-1.0676,-1.0673,-1.067,-1.0667,-1.0664,-1.0661,-1.1284,-1.1873,-1.2429,-1.2952,-1.3443,-1.3901,-1.4326,-1.4718,-1.5075,-1.5397,-1.5685,-1.5936,-1.6151,-1.6329,-1.647,-1.6572,-1.6636,-1.6662,-1.6649,-1.6597,-1.6506,-1.6377,-1.621,-1.6005,-1.5762,-1.5484,-1.5168,-1.4818,-1.4432,-1.4013,-1.3559,-1.3072,-1.2552,-1.1999,-1.1413,-1.0794,-1.0702,-1.0702,-1.0702,-1.0702,-1.0702,0};
      float kneer_loc [] = {0,1.0577,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.1326,1.1917,1.2475,1.2999,1.3491,1.3949,1.4374,1.4765,1.512,1.544,1.5725,1.5973,1.6183,1.6356,1.6491,1.6588,1.6646,1.6666,1.6647,1.6589,1.6493,1.6359,1.6188,1.5979,1.5734,1.5452,1.5135,1.4783,1.4397,1.3977,1.3523,1.3037,1.2517,1.1965,1.138,1.0761,1.067,1.0672,1.0673,1.0674,1.0675,1.0676,1.0676,1.0679,1.0682,1.0684,1.0687,1.0689,1.069,1.0692,1.0693,1.0695,1.0696,1.0697,1.0698,1.0699,1.07,1.07,1.0701,1.0701,1.0701,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0701,1.0701,1.07,1.07,1.0699,1.0698,1.0698,1.0697,1.0696,1.0695,1.0693,1.0692,1.0691,1.0689,1.0687,1.0685,1.0683,1.0681,1.0678,1.0676,1.0676,1.0673,1.067,1.0667,1.0664,1.066,1.1283,1.1871,1.2428,1.2951,1.3443,1.3902,1.4328,1.4721,1.5081,1.5406,1.5696,1.595,1.6167,1.6347,1.6488,1.659,1.6653,1.6676,1.666,1.6603,1.6507,1.6372,1.6198,1.5986,1.5738,1.5453,1.5133,1.4779,1.439,1.3969,1.3514,1.3027,1.2508,1.1957,1.1372,1.0754,1.0666,1.0668,1.0671,1.0674,1.0677,1.0676,1.0679,1.0682,1.0684,1.0686,1.0688,1.069,1.0691,1.0693,1.0694,1.0695,1.0696,1.0697,1.0697,1.0698,1.0698,1.0699,1.0699,1.07,1.07,1.07,1.0701,1.0701,1.0701,1.0701,1.0701,1.0701,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,1.0702,0};
//      float hipPl_loc [] = {0, 0.635074}; // these are the hip initial position in webots
//      float hipPr_loc [] = {0, -0.635074};
      float hipPl_loc [] = {0,0.52886,0.53511,0.53511,0.53511,0.53511,0.53511,0.53511,0.53506,0.53491,0.53468,0.53436,0.53395,0.53346,0.5329,0.53227,0.53156,0.5308,0.52997,0.52908,0.52814,0.52715,0.52611,0.52503,0.52391,0.52276,0.52157,0.52036,0.51912,0.51787,0.5166,0.51531,0.51402,0.51273,0.51143,0.51014,0.50886,0.50759,0.50634,0.50511,0.5039,0.50273,0.50158,0.50048,0.49942,0.4984,0.49744,0.49653,0.49568,0.4949,0.49477,0.49235,0.48999,0.48769,0.48545,0.48326,0.51144,0.53803,0.56319,0.58712,0.60995,0.63184,0.65289,0.67319,0.69282,0.71181,0.73014,0.74776,0.7646,0.78051,0.79534,0.80888,0.82091,0.83122,0.83955,0.84569,0.84943,0.85059,0.84904,0.84468,0.83745,0.82733,0.81436,0.79858,0.78011,0.75905,0.73553,0.70971,0.68173,0.65174,0.61987,0.58623,0.5798,0.57804,0.57623,0.57436,0.57243,0.57286,0.57074,0.56867,0.56665,0.56468,0.56276,0.56088,0.55905,0.55725,0.5555,0.55378,0.5521,0.55045,0.54883,0.54723,0.54566,0.54412,0.54259,0.54108,0.53958,0.5381,0.53662,0.53515,0.53369,0.53222,0.53076,0.52928,0.52781,0.52632,0.52481,0.5233,0.52176,0.5202,0.51861,0.517,0.51536,0.51368,0.51196,0.5102,0.5084,0.50655,0.50466,0.5027,0.50069,0.49862,0.49649,0.49429,0.49477,0.49237,0.49007,0.48786,0.48575,0.48373,0.51212,0.53888,0.56414,0.58797,0.61047,0.63171,0.65174,0.67061,0.68834,0.70496,0.72045,0.7348,0.74796,0.75986,0.77042,0.77955,0.78712,0.79303,0.79715,0.79937,0.79957,0.79765,0.79355,0.78719,0.77854,0.7676,0.75436,0.73886,0.72115,0.70129,0.67934,0.6554,0.62953,0.60181,0.5723,0.54105,0.53619,0.53592,0.53564,0.53534,0.53504,0}; // these are my calculated hip position to keep the body vertical
      float hipPr_loc [] = {0,-0.52886,-0.53511,-0.53511,-0.53511,-0.53511,-0.53511,-0.53511,-0.56628,-0.59575,-0.62359,-0.6499,-0.67472,-0.69813,-0.72015,-0.74082,-0.76014,-0.7781,-0.7947,-0.80988,-0.82359,-0.83577,-0.84633,-0.85516,-0.86217,-0.86725,-0.87029,-0.87121,-0.8699,-0.86631,-0.86038,-0.85209,-0.84143,-0.82841,-0.81306,-0.79546,-0.77565,-0.75373,-0.72978,-0.70389,-0.67615,-0.64664,-0.61543,-0.58257,-0.57692,-0.57604,-0.57519,-0.5744,-0.57366,-0.57297,-0.57286,-0.57074,-0.56867,-0.56665,-0.56468,-0.56276,-0.56088,-0.55905,-0.55725,-0.5555,-0.55378,-0.5521,-0.55045,-0.54883,-0.54723,-0.54566,-0.54412,-0.54259,-0.54108,-0.53958,-0.5381,-0.53662,-0.53515,-0.53369,-0.53222,-0.53076,-0.52928,-0.52781,-0.52632,-0.52481,-0.5233,-0.52176,-0.5202,-0.51861,-0.517,-0.51536,-0.51368,-0.51196,-0.5102,-0.5084,-0.50655,-0.50466,-0.5027,-0.50069,-0.49862,-0.49649,-0.49429,-0.49477,-0.49235,-0.48999,-0.48769,-0.48545,-0.48326,-0.51144,-0.53803,-0.56319,-0.58712,-0.60995,-0.63184,-0.65289,-0.67319,-0.69282,-0.71181,-0.73014,-0.74776,-0.7646,-0.78051,-0.79534,-0.80888,-0.82091,-0.83122,-0.83955,-0.84569,-0.84943,-0.85059,-0.84904,-0.84468,-0.83745,-0.82733,-0.81436,-0.79858,-0.78011,-0.75905,-0.73553,-0.70971,-0.68173,-0.65174,-0.61987,-0.58623,-0.5798,-0.57804,-0.57623,-0.57436,-0.57243,-0.57286,-0.57075,-0.56873,-0.56679,-0.56494,-0.56317,-0.56148,-0.55986,-0.55832,-0.55686,-0.55547,-0.55415,-0.5529,-0.55172,-0.5506,-0.54955,-0.54856,-0.54762,-0.54674,-0.54592,-0.54514,-0.54442,-0.54374,-0.54311,-0.54251,-0.54196,-0.54145,-0.54097,-0.54052,-0.5401,-0.53971,-0.53934,-0.539,-0.53867,-0.53836,-0.53807,-0.53779,-0.53751,-0.53725,-0.53698,-0.53672,-0.53646,-0.53619,-0.53592,-0.53564,-0.53534,-0.53504,0};
      float hipRl_loc [] ={0,0,0.01538,0.030685,0.045839,0.06077,0.075406,0.089678,0.10352,0.11687,0.12966,0.14185,0.15338,0.16419,0.17426,0.18354,0.192,0.1996,0.20632,0.21214,0.21703,0.22098,0.22398,0.22601,0.22708,0.22718,0.2263,0.22446,0.22165,0.21788,0.21318,0.20754,0.201,0.19357,0.18528,0.17616,0.16625,0.15557,0.14418,0.13212,0.11944,0.1062,0.092453,0.078261,0.063691,0.048813,0.033697,0.018416,0.003045,-0.012341,-0.027666,-0.042857,-0.057838,-0.072538,-0.086887,-0.10082,-0.11427,-0.12718,-0.13949,-0.15115,-0.16211,-0.17233,-0.18177,-0.19039,-0.19817,-0.20506,-0.21106,-0.21613,-0.22027,-0.22346,-0.22569,-0.22695,-0.22724,-0.22655,-0.2249,-0.22228,-0.2187,-0.21418,-0.20873,-0.20237,-0.19511,-0.18699,-0.17803,-0.16827,-0.15774,-0.14649,-0.13456,-0.122,-0.10886,-0.09521,-0.0811,-0.0666,-0.051777,-0.036703,-0.021448,-0.0060893,0.0092996,0.024643,0.039866,0.054894,0.069655,0.084079,0.098099,0.11165,0.12467,0.1371,0.1489,0.16,0.17037,0.17997,0.18875,0.1967,0.20377,0.20994,0.2152,0.21953,0.2229,0.22532,0.22677,0.22726,0.22676,0.2253,0.22287,0.21949,0.21515,0.20988,0.2037,0.19662,0.18866,0.17987,0.17026,0.15988,0.14877,0.13697,0.12453,0.1115,0.097949,0.083925,0.069497,0.054732,0.039701,0.024477,0.0091326,-0.0062565,-0.021615,-0.036867,-0.05194,-0.06676,-0.081256,-0.095361,-0.10901,-0.12214,-0.13469,-0.14662,-0.15786,-0.16838,-0.17813,-0.18708,-0.19519,-0.20244,-0.20879,-0.21424,-0.21875,-0.22231,-0.22492,-0.22656,-0.22724,-0.22694,-0.22567,-0.22343,-0.22023,-0.21608,-0.211,-0.20499,-0.19809,-0.1903,-0.18167,-0.17223,-0.162,-0.15103,-0.13936,-0.12704,-0.11413,-0.10067,-0.086733,-0.07238,-0.057676,-0.042693,-0.027501,-0.012174,0};
      float hipRr_loc [] = {0,0,0.01538,0.030685,0.045839,0.06077,0.075406,0.089678,0.10352,0.11687,0.12966,0.14185,0.15338,0.16419,0.17426,0.18354,0.192,0.1996,0.20632,0.21214,0.21703,0.22098,0.22398,0.22601,0.22708,0.22718,0.2263,0.22446,0.22165,0.21788,0.21318,0.20754,0.201,0.19357,0.18528,0.17616,0.16625,0.15557,0.14418,0.13212,0.11944,0.1062,0.092453,0.078261,0.063691,0.048813,0.033697,0.018416,0.003045,-0.012341,-0.027666,-0.042857,-0.057838,-0.072538,-0.086887,-0.10082,-0.11427,-0.12718,-0.13949,-0.15115,-0.16211,-0.17233,-0.18177,-0.19039,-0.19817,-0.20506,-0.21106,-0.21613,-0.22027,-0.22346,-0.22569,-0.22695,-0.22724,-0.22655,-0.2249,-0.22228,-0.2187,-0.21418,-0.20873,-0.20237,-0.19511,-0.18699,-0.17803,-0.16827,-0.15774,-0.14649,-0.13456,-0.122,-0.10886,-0.09521,-0.0811,-0.0666,-0.051777,-0.036703,-0.021448,-0.0060893,0.0092996,0.024643,0.039866,0.054894,0.069655,0.084079,0.098099,0.11165,0.12467,0.1371,0.1489,0.16,0.17037,0.17997,0.18875,0.1967,0.20377,0.20994,0.2152,0.21953,0.2229,0.22532,0.22677,0.22726,0.22676,0.2253,0.22287,0.21949,0.21515,0.20988,0.2037,0.19662,0.18866,0.17987,0.17026,0.15988,0.14877,0.13697,0.12453,0.1115,0.097949,0.083925,0.069497,0.054732,0.039701,0.024477,0.0091326,-0.0062565,-0.021615,-0.036867,-0.05194,-0.06676,-0.081256,-0.095361,-0.10901,-0.12214,-0.13469,-0.14662,-0.15786,-0.16838,-0.17813,-0.18708,-0.19519,-0.20244,-0.20879,-0.21424,-0.21875,-0.22231,-0.22492,-0.22656,-0.22724,-0.22694,-0.22567,-0.22343,-0.22023,-0.21608,-0.211,-0.20499,-0.19809,-0.1903,-0.18167,-0.17223,-0.162,-0.15103,-0.13936,-0.12704,-0.11413,-0.10067,-0.086733,-0.07238,-0.057676,-0.042693,-0.027501,-0.012174,0};
      float tilt_loc [] = {0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3};
      float x1;
      float x2;
      float x3;
      float anklel; 
      float ankler; 
      float footl; 
      float footr;
      float kneel; 
      float kneer;
      float hipPl; 
      float hipPr;
      float hipRl; 
      float hipRr;
      float hipYl;
      float hipYr;
      float neckTilt;
      typedef const boost::shared_ptr<const gazebo::msgs::Vector3d> VectorThreeDPtr;


using namespace std;
namespace gazebo
{
  class PID1Joints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {


      this->model_ = _model;
      // initialize a PID class
      this->target_position_ = 0.0;
      k_ = 0;
//      xp[] = { 0, 1, 2, 3, 4, 5 }; 
//      yp1[] = {-0.2, 0.1, 0.4, 0.7, 1, 1.2};
      
// leg joints
      this->pid_j_ankle2_r.Init(10, 0.01, 4, 0, 0, 25, -25);
      this->pid_j_ankle2_r.SetCmd(this->target_position_);
      this->ankle2_r_joint_ = this->model_->GetJoint("j_ankle2_r");

      this->pid_j_ankle1_r.Init(20, 0.0, 9, 0, 0, 25, -25);
      this->pid_j_ankle1_r.SetCmd(this->target_position_);
      this->ankle1_r_joint_ = this->model_->GetJoint("j_ankle1_r");

      this->pid_j_tibia_r.Init(20, 0.0, 1, 0, 0, 25, -25);
      this->pid_j_tibia_r.SetCmd(this->target_position_);
      this->tibia_r_joint_ = this->model_->GetJoint("j_tibia_r");

      this->pid_j_thigh2_r.Init(20, 0.0, 9, 0, 0, 25, -25);
      this->pid_j_thigh2_r.SetCmd(this->target_position_);
      this->thigh2_r_joint_ = this->model_->GetJoint("j_thigh2_r");

      this->pid_j_thigh1_r.Init(45, 0.5, 18, 0, 0, 25, -25);
      this->pid_j_thigh1_r.SetCmd(this->target_position_);
      this->thigh1_r_joint_ = this->model_->GetJoint("j_thigh1_r");
// left leg
      this->pid_j_ankle2_l.Init(10, 0.01, 4, 0, 0, 25, -25);
      this->pid_j_ankle2_l.SetCmd(this->target_position_);
      this->ankle2_l_joint_ = this->model_->GetJoint("j_ankle2_l");

      this->pid_j_ankle1_l.Init(20, 0.0, 9, 0, 0, 25, -25);
      this->pid_j_ankle1_l.SetCmd(this->target_position_);
      this->ankle1_l_joint_ = this->model_->GetJoint("j_ankle1_l");

      this->pid_j_tibia_l.Init(20, 0.00, 1, 0, 0, 25, -25);
      this->pid_j_tibia_l.SetCmd(this->target_position_);
      this->tibia_l_joint_ = this->model_->GetJoint("j_tibia_l");

      this->pid_j_thigh2_l.Init(20, 0.0, 9, 0, 0, 25, -25);
      this->pid_j_thigh2_l.SetCmd(this->target_position_);
      this->thigh2_l_joint_ = this->model_->GetJoint("j_thigh2_l");

      this->pid_j_thigh1_l.Init(45, 0.5, 18, 0, 0, 25, -25);
      this->pid_j_thigh1_l.SetCmd(this->target_position_);
      this->thigh1_l_joint_ = this->model_->GetJoint("j_thigh1_l");
// left shoulder and upper limb
      this->pid_j_shoulder_l.Init(2, 0, 0, 0, 0, 25, -25);
      this->pid_j_shoulder_l.SetCmd(this->target_position_);
      this->shoulder_l_joint_ = this->model_->GetJoint("j_shoulder_l");

      this->pid_j_high_arm_l.Init(2, 0, 0, 0, 0, 25, -25);
      this->pid_j_high_arm_l.SetCmd(this->target_position_);
      this->high_arm_l_joint_ = this->model_->GetJoint("j_high_arm_l");

      this->pid_j_low_arm_l.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_low_arm_l.SetCmd(this->target_position_);
      this->low_arm_l_joint_ = this->model_->GetJoint("j_low_arm_l");

      this->pid_j_wrist_l.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_wrist_l.SetCmd(this->target_position_);
      this->wrist_l_joint_ = this->model_->GetJoint("j_wrist_l");

      this->pid_j_gripper_l.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_gripper_l.SetCmd(this->target_position_);
      this->gripper_l_joint_ = this->model_->GetJoint("j_gripper_l");

// right hand

      this->pid_j_shoulder_r.Init(2, 0, .5, 0, 0, 25, -25);
      this->pid_j_shoulder_r.SetCmd(this->target_position_);
      this->shoulder_r_joint_ = this->model_->GetJoint("j_shoulder_r");

      this->pid_j_high_arm_r.Init(2, 0, .5, 0, 0, 25, -25);
      this->pid_j_high_arm_r.SetCmd(this->target_position_);
      this->high_arm_r_joint_ = this->model_->GetJoint("j_high_arm_r");

      this->pid_j_low_arm_r.Init(1, 0, 0.5, 0, 0, 25, -25);
      this->pid_j_low_arm_r.SetCmd(this->target_position_);
      this->low_arm_r_joint_ = this->model_->GetJoint("j_low_arm_r");

      this->pid_j_wrist_r.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_wrist_r.SetCmd(this->target_position_);
      this->wrist_r_joint_ = this->model_->GetJoint("j_wrist_r");

      this->pid_j_gripper_r.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_gripper_r.SetCmd(this->target_position_);
      this->gripper_r_joint_ = this->model_->GetJoint("j_gripper_r");

      this->pid_j_pan.Init(1, 0, 0, 0, 0, 25, -25);
      this->pid_j_pan.SetCmd(this->target_position_);
      this->pan_joint_ = this->model_->GetJoint("j_pan");

      this->pid_j_tilt.Init(10, 0, 3, 0, 0, 25, -25);
      this->pid_j_tilt.SetCmd(this->target_position_);
      this->tilt_joint_ = this->model_->GetJoint("j_tilt");

      this->pid_j_pelvis_l.Init(275, 0.0, 20, 0, 0, 25, -25);
      this->pid_j_pelvis_l.SetCmd(this->target_position_);
      this->pelvis_l_joint_ = this->model_->GetJoint("j_pelvis_l");

      this->pid_j_pelvis_r.Init(275, 0.0, 20, 0, 0, 25, -25);
      this->pid_j_pelvis_r.SetCmd(this->target_position_);
      this->pelvis_r_joint_ = this->model_->GetJoint("j_pelvis_r");


      this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateStart(
        boost::bind(&PID1Joints::UpdatePID, this));
    }
    void UpdatePID()
    {
      common::Time current_time = this->model_->GetWorld()->GetSimTime();

//      if (k_ == 0)
//      {
//        myfile1.open("pelvisY13.csv");
//      }         
        k_ = k_ + 1;

        /*if (k_==1)
        {gazebo::transport::NodePtr node(new gazebo::transport::Node());
           node->Init();
           gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose_example1", &PID1Joints::cb,this);
}*/
        double error;
        double dt    = current_time.Double()
                   - this->last_update_time_.Double();

        if (x1<1)
        {
          footl = lip(x1,xp,footl_loc);
          footr = lip(x1,xp,footr_loc);
          kneel = lip(x1,xp,kneel_loc);
          kneer = lip(x1,xp,kneer_loc);
          hipPl = lip(x1,xp,hipPl_loc);
          hipPr = lip(x1,xp,hipPr_loc);
          neckTilt = lip(x1,xp,tilt_loc);
//          hipYl = lip(x1,xp,hipYl_loc);
//          hipYr = lip(x1,xp,hipYr_loc);

//          gzdbg << "footl:{" << footl << "  footr:{" << footr << "}  x : {" << x1 << "} \n";
//          gzdbg << "kneel:{" << kneel << "  kneer:{" << kneer << "}  x : {" << x1 << "} \n";
//          gzdbg << "hipPl:{" << hipPl << "  hipPr:{" << hipPr << "}  x : {" << x1 << "} \n";

          ankle1_l(footl,dt);
          ankle1_r(footr,dt);
          tibia_l(kneel,dt);
          tibia_r(kneer,dt);
          thigh2_l(hipPl,dt);
          thigh2_r(hipPr,dt);
          tilt(neckTilt,dt);

          ankle2_l(0.0,dt);
          ankle2_r(0.0,dt);
          thigh1_l(0.0,dt);
          thigh1_r(0.0,dt);
          pelvis_l(0.0,dt);
          pelvis_r(0.0,dt);
          SubChecker = 1;
      
        }
        else
        {

          if (SubChecker==1)
          {
            gazebo::transport::NodePtr node(new gazebo::transport::Node());
             node->Init();
             gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose_example1", &PID1Joints::cb,this);
             a =1;
             while(a!=2)
             {
              gazebo::common::Time::MSleep(10);
              }
              stepsToWalk = (distFromBall - .1)/0.025;
              std::cout << "Steps To Walk :" << stepsToWalk << "\n";}
          SubChecker = SubChecker+ 1;

          footl = lip(x1,xp,footl_loc);
          footr = lip(x1,xp,footr_loc);
          kneel = lip(x1,xp,kneel_loc);
          kneer = lip(x1,xp,kneer_loc);
          hipPl = lip(x1,xp,hipPl_loc);
          hipPr = lip(x1,xp,hipPr_loc);
         
          // hip swing movement
          anklel = lip(x1,xp,anklel_loc);
          ankler = lip(x1,xp,ankler_loc);
          hipRl = lip(x1,xp,hipRl_loc);
          hipRr = lip(x1,xp,hipRr_loc);
         
//          hipYl = lip(x1,xp,hipYl_loc);
//          hipYr = lip(x1,xp,hipYr_loc);
          neckTilt = lip(x1,xp,tilt_loc);
//          gzdbg << "footl:{" << footl << "  footr:{" << footr << "}  x : {" << x1 << "} \n";
//          gzdbg << "kneel:{" << kneel << "  kneer:{" << kneer << "}  x : {" << x1 << "} \n";
//          gzdbg << "hipPl:{" << hipPl << "  hipPr:{" << hipPr << "}  x : {" << x1 << "} \n";
//          gzdbg << "anklel:{" << anklel << "  ankler:{" << ankler << "}  x : {" << x1 << "} \n";
//          gzdbg << "hipRl:{" << hipRl << "  hipRr:{" << hipRr << "}  x : {" << x1 << "} \n";

          ankle2_l(anklel,dt);
          ankle2_r(ankler,dt);

          ankle1_l(footl,dt);
          ankle1_r(footr,dt);
          tibia_l(kneel,dt);
          tibia_r(kneer,dt);
          thigh2_l(hipPl,dt);
          thigh2_r(hipPr,dt);

          thigh1_l(hipRl,dt);
          thigh1_r(hipRr,dt);

          pelvis_l(0.0,dt);
          pelvis_r(0.0,dt);
          tilt(neckTilt,dt);
// 	  myfile1 << hipYl << "," << this->pelvis_l_joint_->GetAngle(0).Radian() << "\n";
        }


// left arm
      error = this->shoulder_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_shoulder_l.Update(error, dt);
      this->shoulder_l_joint_->SetForce(0, this->pid_j_shoulder_l.GetCmd());


      error = this->high_arm_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_high_arm_l.Update(error, dt);
      this->high_arm_l_joint_->SetForce(0, this->pid_j_high_arm_l.GetCmd());

      error = this->low_arm_l_joint_->GetAngle(0).Radian()
                   - target_position_ ;
      this->pid_j_low_arm_l.Update(error, dt);
      this->low_arm_l_joint_->SetForce(0, this->pid_j_low_arm_l.GetCmd());


      error = this->wrist_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_wrist_l.Update(error, dt);
      this->wrist_l_joint_->SetForce(0, this->pid_j_wrist_l.GetCmd());
      

      
      error = this->gripper_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_gripper_l.Update(error, dt);
      this->gripper_l_joint_->SetForce(0, this->pid_j_gripper_l.GetCmd());


// right arm 
      error = this->shoulder_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_shoulder_r.Update(error, dt);
      this->shoulder_r_joint_->SetForce(0, this->pid_j_shoulder_r.GetCmd());

      error = this->high_arm_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_high_arm_r.Update(error, dt);
      this->high_arm_r_joint_->SetForce(0, this->pid_j_high_arm_r.GetCmd());

       error = this->low_arm_r_joint_->GetAngle(0).Radian()
                 - target_position_ ;
       this->pid_j_low_arm_r.Update(error, dt);
       this->low_arm_r_joint_->SetForce(0, this->pid_j_low_arm_r.GetCmd());
      error = this->wrist_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_wrist_r.Update(error, dt);
      this->wrist_r_joint_->SetForce(0, this->pid_j_wrist_r.GetCmd());
      
      error = this->gripper_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_gripper_r.Update(error, dt);
      this->gripper_r_joint_->SetForce(0, this->pid_j_gripper_r.GetCmd());
// head and pelvis 

      error = this->pan_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_pan.Update(error, dt);
      this->pan_joint_->SetForce(0, this->pid_j_pan.GetCmd());

      if ((x1>144)&&(x1<144.15))
       {std::cout << "I just incremented\n";
        stepCount = stepCount +1;
        if (stepCount < stepsToWalk)
          {x1 = 49;}}
      std::cout << stepCount << "\n";
      if (x1<1)
       x1 = x1 + 0.001;
      else if ((x1+0.1)<189.9)
       {x1 = x1 + 0.1;
       std::cout << x1 << "\n";}
      else if ((x1+0.001)<190.998)
       {x1 = x1 + 0.001;
       std::cout << x1 << "\n";}
      else
        {x1 = 190.998;
         std::cout << "I was just Called\n";}
      /*if (x1<1)
       x1 = x1 + 0.001;
      else
        x1 = 1;*/
      this->last_update_time_ = current_time;
//      gzdbg << x1 + x2;
    }



float lip(float x,float xp[],float yp[])
{
  float y;
  float xp0;
  float yp0;
  float xp1;
  float yp1;
  for (int i =0; i<=192; i++)
  {
    if (xp[i]<=x)
    {
      xp0 = xp[i];
      yp0 = yp[i];
    }
    if (xp[i]>x)
    {
      xp1 = xp[i];
      yp1 = yp[i];
      break;
    }
  }
//  gzdbg << "xp0:{" << xp0  << "} \n";
//  gzdbg << "xp1:{" << xp1  << "} \n";
//  gzdbg << "yp0:{" << yp0  << "} \n";
//  gzdbg << "yp1:{" << yp1  << "} \n";
  float m = (yp1-yp0)/(xp1-xp0);
  y = (yp0 +(m*(x-xp0)));
  return (y);
}

void ankle2_l(float position, double dt1)
{
  float error1;
  this->pid_j_ankle2_l.SetCmd(position);
  error1 = this->ankle2_l_joint_->GetAngle(0).Radian() - position ;
//  gzdbg <<  "Current anklel : " << this->ankle2_l_joint_->GetAngle(0).Radian() << "\n";
  this->pid_j_ankle2_l.Update(error1, dt1);
  this->ankle2_l_joint_->SetForce(0, this->pid_j_ankle2_l.GetCmd());
}

void ankle2_r(float position, double dt1)
{
  float error1;
  this->pid_j_ankle2_r.SetCmd(position);
  error1 = this->ankle2_r_joint_->GetAngle(0).Radian() - position ;
//  gzdbg <<  "Current ankler : " << this->ankle2_r_joint_->GetAngle(0).Radian() << "\n";
  this->pid_j_ankle2_r.Update(error1, dt1);
  this->ankle2_r_joint_->SetForce(0, this->pid_j_ankle2_r.GetCmd());
}

void ankle1_l(float position, double dt1)
{
  float error1;
  this->pid_j_ankle1_l.SetCmd(position);
  error1 = this->ankle1_l_joint_->GetAngle(0).Radian() - position ;
//  gzdbg <<  "Current footl : " << this->ankle1_l_joint_->GetAngle(0).Radian() << "\n";
  this->pid_j_ankle1_l.Update(error1, dt1);
  this->ankle1_l_joint_->SetForce(0, this->pid_j_ankle1_l.GetCmd());
}

void ankle1_r(float position, double dt1)
{
  float error1;
  this->pid_j_ankle1_r.SetCmd(position);
  error1 = this->ankle1_r_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current footr : " << this->ankle1_r_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_ankle1_r.Update(error1, dt1);
  this->ankle1_r_joint_->SetForce(0, this->pid_j_ankle1_r.GetCmd());
}

void tibia_l(float position, double dt1)
{
  float error1;
  this->pid_j_tibia_l.SetCmd(position);
  error1 = this->tibia_l_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current kneel : " << this->tibia_l_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_tibia_l.Update(error1, dt1);
  this->tibia_l_joint_->SetForce(0, this->pid_j_tibia_l.GetCmd());
}

void tibia_r(float position, double dt1)
{
  float error1;
  this->pid_j_tibia_r.SetCmd(position);
  error1 = this->tibia_r_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current kneer : " << this->tibia_r_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_tibia_r.Update(error1, dt1);
  this->tibia_r_joint_->SetForce(0, this->pid_j_tibia_r.GetCmd());
}

void thigh2_l(float position, double dt1)
{
  float error1;
  this->pid_j_thigh2_l.SetCmd(position);
  error1 = this->thigh2_l_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current hipPl : " << this->thigh2_l_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_thigh2_l.Update(error1, dt1);
  this->thigh2_l_joint_->SetForce(0, this->pid_j_thigh2_l.GetCmd());
}

void thigh2_r(float position, double dt1)
{
  float error1;
  this->pid_j_thigh2_r.SetCmd(position);
  error1 = this->thigh2_r_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current hipPr : " << this->thigh2_r_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_thigh2_r.Update(error1, dt1);
  this->thigh2_r_joint_->SetForce(0, this->pid_j_thigh2_r.GetCmd());
}

void thigh1_l(float position, double dt1)
{
  float error1;
  this->pid_j_thigh1_l.SetCmd(position);
  error1 = this->thigh1_l_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current hipRl : " << this->thigh1_l_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_thigh1_l.Update(error1, dt1);
  this->thigh1_l_joint_->SetForce(0, this->pid_j_thigh1_l.GetCmd());
}

void thigh1_r(float position, double dt1)
{
  float error1;
  this->pid_j_thigh1_r.SetCmd(position);
  error1 = this->thigh1_r_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current hipRr : " << this->thigh1_r_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_thigh1_r.Update(error1, dt1);
  this->thigh1_r_joint_->SetForce(0, this->pid_j_thigh1_r.GetCmd());
}

void pelvis_r(float position, double dt1)
{
  float error1;
  this->pid_j_pelvis_r.SetCmd(position);
  error1 = this->pelvis_r_joint_->GetAngle(0).Radian() - position;
  this->pid_j_pelvis_r.Update(error1, dt1);
  this->pelvis_r_joint_->SetForce(0, this->pid_j_pelvis_r.GetCmd());
}

void pelvis_l(float position, double dt1)
{
  float error1;
  this->pid_j_pelvis_l.SetCmd(position);
  error1 = this->pelvis_l_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current hipRr : " << this->thigh1_r_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_pelvis_l.Update(error1, dt1);
  this->pelvis_l_joint_->SetForce(0, this->pid_j_pelvis_l.GetCmd());
}

void tilt(float position, double dt1)
{
  float error1;
  this->pid_j_tilt.SetCmd(position);
  error1 = this->tilt_joint_->GetAngle(0).Radian() - position;
//  gzdbg <<  "Current position : " << this->tilt_joint_->GetAngle(0).Radian()<< "\n";
  this->pid_j_tilt.Update(error1, dt1);
  this->tilt_joint_->SetForce(0, this->pid_j_tilt.GetCmd());
}

      public: void cb(VectorThreeDPtr &_msg)
      {
        // Dump the message contents to stdout.
        std::cout << "I am Recieving ::" ;
        std::cout << "x : " <<_msg->x() << "\n";
        std::cout << "y : " <<_msg->y() << "\n";
        std::cout << "Distance : " <<_msg->z() << "\n";
        x_1 = _msg->x();
        y_1 = _msg->y();
        distFromBall = _msg->z();
        a = 2;
      }

// PID controllers for each joint :

// head

    common::PID pid_j_pan;
    common::PID pid_j_tilt;

// pelvis

    common::PID pid_j_pelvis_l;
    common::PID pid_j_pelvis_r;


// leg joints
    common::PID pid_j_ankle2_r;
    common::PID pid_j_ankle1_r;
    common::PID pid_j_tibia_r;
    common::PID pid_j_thigh2_r;
    common::PID pid_j_thigh1_r;

    common::PID pid_j_ankle2_l;
    common::PID pid_j_ankle1_l;
    common::PID pid_j_tibia_l;
    common::PID pid_j_thigh2_l;
    common::PID pid_j_thigh1_l;

// left arm 
    common::PID pid_j_shoulder_l;
    common::PID pid_j_high_arm_l;
    common::PID pid_j_low_arm_l;
    common::PID pid_j_wrist_l;
    common::PID pid_j_gripper_l;

    common::PID pid_j_shoulder_r;
    common::PID pid_j_high_arm_r;
    common::PID pid_j_low_arm_r;
    common::PID pid_j_wrist_r;
    common::PID pid_j_gripper_r;


// pid controllers for the leg joints

    double target_position_;
    double k_;
    double elbow_position;
    double x_1;
    double y_1;
    double distFromBall;
    double SubChecker;
    double a;
    int stepCount;
    int stepsToWalk;
//    ofstream myfile1;
// ** Pointers for each joints

// head
    physics::JointPtr pan_joint_;
    physics::JointPtr tilt_joint_;

// pelvis
    physics::JointPtr pelvis_l_joint_;
    physics::JointPtr pelvis_r_joint_;


// legs
    physics::JointPtr ankle2_r_joint_;
    physics::JointPtr ankle1_r_joint_;
    physics::JointPtr tibia_r_joint_;
    physics::JointPtr thigh2_r_joint_;
    physics::JointPtr thigh1_r_joint_;

    physics::JointPtr ankle2_l_joint_;
    physics::JointPtr ankle1_l_joint_;
    physics::JointPtr tibia_l_joint_;
    physics::JointPtr thigh2_l_joint_;
    physics::JointPtr thigh1_l_joint_;

// arm 

    physics::JointPtr shoulder_l_joint_;
    physics::JointPtr high_arm_l_joint_;
    physics::JointPtr low_arm_l_joint_;
    physics::JointPtr wrist_l_joint_;
    physics::JointPtr gripper_l_joint_;

    physics::JointPtr shoulder_r_joint_;
    physics::JointPtr high_arm_r_joint_;
    physics::JointPtr low_arm_r_joint_;
    physics::JointPtr wrist_r_joint_;
    physics::JointPtr gripper_r_joint_;

    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PID1Joints)
}
