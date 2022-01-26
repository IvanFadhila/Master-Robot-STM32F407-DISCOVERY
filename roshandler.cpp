/*
 * roshandler.cpp
 *
 *  Created on: Mar 23, 2021
 *      Author: Fattah .Alf
 */
#include <roshandler.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>

volatile float direction[4] = { 0, 0, 0, 0 };
volatile float setpoint[4] = { 0, 0, 0, 0 };
volatile int position = 0;
volatile float rpm[4] = { 0., 0., 0., 0. };
volatile float output[4] = { 0., 0., 0., 0. };

//float poly_fr[] = { 98.4238016421869, 0.510836315011864, 0.00200914235082075,-0.00000247745059990939, 0.000000000948099064298469 };
//float poly_br[] = {126.384580918238, 0.995987220939329, -0.00278685688145045, 0.0000136653488707105, -0.0000000211519840245322, 0.0000000000104184423021175};
////float poly_br[] = {96.8084361378418,0.54266594808806,0.00201044418067922,-0.00000254548458511026,0.00000000096904283424679};
//float poly_fl[] = {96.8084361378418,0.54266594808806,0.00201044418067922,-0.00000254548458511026,0.00000000096904283424679};
//float poly_bl[] = {116.772567673639,0.189103449226447,0.00358925032208785,-0.00000509453564328442,0.0000000023239351131012};
//
//float polyn_fr[] = {-85.0570942610212,0.88315868528041,-0.00114061012463333,-0.00000118677221247197,-0.000000000189460998635842};
//float polyn_br[] = {-123.71755123881,0.862483894121515,0.00188780633610614,0.0000113251918579058,0.000000018379158546596,0.00000000000924017469972154};
////float polyn_br[] = {-138.067039291544,0.214157450347317,-0.00345263414496891,-0.00000489339852647618,-0.0000000022024757608141};
//float polyn_fl[] = {-138.512972441212,0.401282118873958,-0.0030383244298189,-0.00000452167697147673,-0.00000000206807416101967};
//float polyn_bl[] = {-117.28478669862,0.517019250611982,-0.00226260323916755,-0.00000285450370738228,-0.00000000106518794520166};

//float batas_rpm = 250;
//double atasPlus[4][3] = {{-6.92133102093808,1.39633706040313,-0.000312092127625931},{-9.47834426064766,1.36232062842081,-0.000262418921189015},{-17.6680381662257,1.43143408360117,-0.000334994117419187},{13.5950798794384,1.43267164514847,-0.000394034739738073}};
//double bawahPlus[4][5] = {{126.316058979872,-0.650660828170713,0.0196383136263833,-0.000120463955254975,0.000000289290268431452},
//						{183.05697553536,-3.17222291511868,0.0538876841495866,-0.000295521012873213,0.000000577890162840075},
//						{62.2424700196303,3.40667745221619,-0.0531197345603726,0.000374090964116719,-0.00000084974877080002},
//						{125.22560459813,1.4585535556073,-0.0129823584457485,0.0000761170009997779,-0.000000128843744660703}};
//
//
//
//
//double bawahMinus[4][5] = {{-125.048423966385,1.78803613676977,0.0264138743837288,0.000236171833534684,0.000000673970353204553},
//						  {-42.7833330659058,2.69199047046307,0.0255349788674543,0.000163670410155951,0.000000367482956458508},
//						  {-109.622493276597,1.17645868825241,0.0122651719916557,0.000112283142201164,0.000000302080562347235},
//						  {-125.143312160064,0.836111923670448,-0.00119013279404749,-0.0000446250437141784,-0.000000209676987701674}};
//double atasMinus[4][3] = {{-20.9474871394102,1.4972254531272,0.000436963040622311},{-5.14474721983139,1.5045976475652,0.000331354265650862},{-4.5246110472368,1.46579212132518,0.000347802340188311},{2.64599200153734,1.46434236096695,0.00038936132638104}};

double plus[4][5] = {{93.2701310476527,0.626860866591994,0.00178222802778684,-0.00000180943350702257,0.000000000328602089290757},
					{91.9746012650567,0.679086195649278,0.00147161755979102,-0.00000107178443360446,-0.000000000158765001229399},
					{105.497021578623,0.373263968665546,0.00307456378946635,-0.00000396880974504809,0.00000000150877119818233},
//					{144.212834839538,0.465395517321831,0.00289026909571114,-0.00000431811143843963,0.00000000198271261972356}};
//					{91.9746012650567,0.679086195649278,0.00147161755979102,-0.00000107178443360446,-0.000000000158765001229399}};
//					{105.497021578623,0.373263968665546,0.00307456378946635,-0.00000396880974504809,0.00000000150877119818233}};
//					{93.2701310476527,0.626860866591994,0.00178222802778684,-0.00000180943350702257,0.000000000328602089290757}};
					{140.260995731909,0.347678501130237,0.00309081818160137,-0.00000440644071725958,0.00000000200440620157206}};

double plus_khusus[] = {126.38814612625,1.24533754460372,-0.00465808416510098};
double batas_rpmM[] = {-91.24,-121.66,-89.86,-88.48};

double minus[4][5] = {{-95.8549311755911,0.595282967150909,-0.00245395056786048,-0.00000352468113329529,-0.00000000153745013058549},//FR
//					 {-138.173453354976,0.348212038900049,-0.00353047077745759,-0.00000556296654694796,-0.00000000269919819195317},//FL
					 {-95.8549311755911,0.595282967150909,-0.00245395056786048,-0.00000352468113329529,-0.00000000153745013058549},//FR
					 {-123.676385457664,0.406421360282412,-0.00540432294924769,-0.0000101947132216104,-0.00000000603700684963599},//BL
//					 {-143.187576537346,0.279727396608849,-0.00388276006601658,-0.00000635604870178088,-0.00000000327342604070227},//BR
//					 {-135.50864836061,0.292558018965875,-0.00294110595029005,-0.0000036994464692763,-0.00000000146367037651674},//BR
					 {-135.50864836061,0.292558018965875,-0.00294110595029005,-0.0000036994464692763,-0.00000000146367037651674}};//BR
//					 {-95.8549311755911,0.595282967150909,-0.00245395056786048,-0.00000352468113329529,-0.00000000153745013058549}};//BR


//double minus_khusus[4][3] = {{-126.013608170714,1.12657279782461,0.00351157402932046},
//							{-64.7076595189059,1.42	908731961447,0.00311216095101184},
//							{-116.206812209,0.787828863568734,-0.000652372558769381},
//							{-137.5,-4.69012780666064,-0.109375}};

double minus_khusus[4][3] = {{-126.013608170714,1.12657279782461,0.00351157402932046},
							{-125,0.880153645347941,0.00190909737806782},
							{-116.206812209,0.787828863568734,-0.000652372558769381},
							{-64.7076595189059,1.42908731961447,0.00311216095101184}};

void pwm_callback(const geometry_msgs::Quaternion &data) {
	setpoint[0] = data.x;
	setpoint[1] = data.y;
	setpoint[2] = data.z;
	setpoint[3] = data.w;
}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Quaternion> pwm_sub("mini_robot/setpoint",
		&pwm_callback);
geometry_msgs::Quaternion rpm_msg;
ros::Publisher rpm_init("mini_robot/rpm", &rpm_msg);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void setup(void) {
	nh.initNode();
	nh.advertise(rpm_init);
	nh.subscribe(pwm_sub);
}

void loop(void) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	nh.spinOnce();
	HAL_Delay(10);
}

void linearMotor(float motor[4]) {
	float pwm[4] = {setpoint[0],setpoint[1],setpoint[2],setpoint[3]};
	for(int i = 0; i < 4; i++){
		if(pwm[i] > 0){
			direction[i] = (plus[i][0] * 1) + (pwm[i] * plus[i][1]) + ((pwm[i] * pwm[i]) * plus[i][2]) + ((pwm[i] * pwm[i] * pwm[i]) * plus[i][3]) + ((pwm[i] * pwm[i] * pwm[i] * pwm[i]) * plus[i][4]);
//			if(i == 1 && pwm[i] < 109.22){
//				direction[i] = (plus_khusus[0] * 1) + (pwm[i] * plus_khusus[1]) + ((pwm[i]  * pwm[i]) * plus_khusus[2]);
//			}
		}
		else if(pwm[i] < 0){
//			if(pwm[i] <= batas_rpmM[i]){
				direction[i] = (minus[i][0] * 1) + (pwm[i] * minus[i][1]) + ((pwm[i] * pwm[i]) * minus[i][2]) + ((pwm[i] * pwm[i] * pwm[i]) * minus[i][3]) + ((pwm[i] * pwm[i] * pwm[i] * pwm[i]) * minus[i][4]);
//			}
//			else{
//				direction[i] = (minus_khusus[i][0] * 1) + (pwm[i] * minus_khusus[i][1]) + ((pwm[i]  * pwm[i]) * minus_khusus[i][2]);
//			}
		}
		else{
			direction[i] = 0;
		}
	}
}

//void linearMotor(float pwm[4]) {
//	for (int i = 0; i < 4; i++) {
//			if (pwm[i] > 0) {
//				if (pwm[i] <= batas_rpm) {
//					direction[i] = (bawahPlus[i][0] * 1) + (pwm[i] * bawahPlus[i][1]) + ((pwm[i] * pwm[i]) * bawahPlus[i][2]) + ((pwm[i] * pwm[i] * pwm[i]) * bawahPlus[i][3]) + ((pwm[i] * pwm[i] * pwm[i] * pwm[i]) * bawahPlus[i][4]);
//				} else {
//					direction[i] = (atasPlus[i][0] * 1) + (pwm[i] * atasPlus[i][1]) + ((pwm[i]  * pwm[i]) * atasPlus[i][2]);
//				}
//			} else if (pwm[i] < 0) {
//				if (pwm[i] >= (batas_rpm * -1)) {
//					direction[i] = (bawahMinus[i][0] * 1) + (pwm[i] * bawahMinus[i][1]) + ((pwm[i] * pwm[i]) * bawahMinus[i][2]) + ((pwm[i] * pwm[i] * pwm[i]) * bawahMinus[i][3]) + ((pwm[i] * pwm[i] * pwm[i] * pwm[i]) * bawahMinus[i][4]);
//				} else {
//					direction[i] = (atasMinus[i][0] * 1) + (pwm[i] * atasMinus[i][1]) + ((pwm[i]  * pwm[i]) * atasMinus[i][2]);
//				}
//			} else {
//				direction[i] = 0;
//			}
//		}
//}

//void linearisasi(void) {
//	// FR
//	if(setpoint[1] > 0){
//			direction[1] = (poly_fr[0] * 1) + (poly_fr[1] * setpoint[1]) + (poly_fr[2] * (setpoint[1] * setpoint[1])) + (poly_fr[3] * (setpoint[1] * setpoint[1] * setpoint[1])) + (poly_fr[4] * (setpoint[1] * setpoint[1] * setpoint[1] * setpoint[1]));
//	}
//	else if(setpoint[1] < 0) {
//			direction[1] = (polyn_fr[0] * 1) + (polyn_fr[1] * setpoint[1]) + (polyn_fr[2] * (setpoint[1] * setpoint[1])) + (polyn_fr[3] * (setpoint[1] * setpoint[1] * setpoint[1])) + (polyn_fr[4] * (setpoint[1] * setpoint[1] * setpoint[1] * setpoint[1]));
//	}
//	else{
//		direction[1] = 0;
//	}
//
//	// BR
//	if(setpoint[3] > 0){
//			direction[3] = (poly_br[0] * 1) + (poly_br[1] * setpoint[3]) + (poly_br[2] * (setpoint[3] * setpoint[3])) + (poly_br[3] * (setpoint[3] * setpoint[3] * setpoint[3])) + (poly_br[4] * (setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3])) + (poly_br[5] * (setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3]));
////			direction[3] += 62.;
//	}
//	else if(setpoint[3] < 0) {
//			direction[3] = (polyn_br[0] * 1) + (polyn_br[1] * setpoint[3]) + (polyn_br[2] * (setpoint[3] * setpoint[3])) + (polyn_br[3] * (setpoint[3] * setpoint[3] * setpoint[3])) + (polyn_br[4] * (setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3])) + (polyn_br[5] * (setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3] * setpoint[3]));
////			direction[3] -= 25;
//	}
//	else{
//		direction[3] = 0;
//	}
//
//	// FL
//	if(setpoint[0] > 0){
//			direction[0] = (poly_fl[0] * 1) + (poly_fl[1] * setpoint[0]) + (poly_fl[2] * (setpoint[0] * setpoint[0])) + (poly_fl[3] * (setpoint[0] * setpoint[0] * setpoint[0])) + (poly_fl[4] * (setpoint[0] * setpoint[0] * setpoint[0] * setpoint[0]));
//	}
//	else if(setpoint[0] < 0) {
//			direction[0] = (polyn_fl[0] * 1) + (polyn_fl[1] * setpoint[0]) + (polyn_fl[2] * (setpoint[0] * setpoint[0])) + (polyn_fl[3] * (setpoint[0] * setpoint[0] * setpoint[0])) + (polyn_fl[4] * (setpoint[0] * setpoint[0] * setpoint[0] * setpoint[0]));
//	}
//	else{
//		direction[0] = 0;
//	}
//
//	//BL
//	if(setpoint[2] > 0){
//			direction[2] = (poly_bl[0] * 1) + (poly_bl[1] * setpoint[2]) + (poly_bl[2] * (setpoint[2] * setpoint[2])) + (poly_bl[3] * (setpoint[2] * setpoint[2] * setpoint[2])) + (poly_bl[4] * (setpoint[2] * setpoint[2] * setpoint[2] * setpoint[2]));
//	}
//	else if(setpoint[2] < 0){
//			direction[2] = (polyn_bl[0] * 1) + (polyn_bl[1] * setpoint[2]) + (polyn_bl[2] * (setpoint[2] * setpoint[2])) + (polyn_bl[3] * (setpoint[2] * setpoint[2] * setpoint[2])) + (polyn_bl[4] * (setpoint[2] * setpoint[2] * setpoint[2] * setpoint[2]));
//	}
//	else{
//		direction[2] = 0;
//	}
//}
//
//void linearisasi_pid(float dir[4]) {
//	// FR
//	if(dir[1] > 0){
//			direction[1] = (poly_fr[0] * 1) + (poly_fr[1] * dir[1]) + (poly_fr[2] * (dir[1] * dir[1])) + (poly_fr[3] * (dir[1] * dir[1] * dir[1])) + (poly_fr[4] * (dir[1] * dir[1] * dir[1] * dir[1]));
//	}
//	else if(dir[1] < 0) {
//			direction[1] = (polyn_fr[0] * 1) + (polyn_fr[1] * dir[1]) + (polyn_fr[2] * (dir[1] * dir[1])) + (polyn_fr[3] * (dir[1] * dir[1] * dir[1])) + (polyn_fr[4] * (dir[1] * dir[1] * dir[1] * dir[1]));
//	}
//	else{
//		direction[1] = 0;
//	}
//
//	// BR
//	if(dir[3] > 0){
//			direction[3] = (poly_br[0] * 1) + (poly_br[1] * dir[3]) + (poly_br[2] * (dir[3] * dir[3])) + (poly_br[3] * (dir[3] * dir[3] * dir[3])) + (poly_br[4] * (dir[3] * dir[3] * dir[3] * dir[3]));// + (poly_br[5] * (dir[3] * dir[3] * dir[3] * dir[3] * dir[3]));
//			direction[3] += 62.;
//	}
//	else if(dir[3] < 0) {
//			direction[3] = (polyn_br[0] * 1) + (polyn_br[1] * dir[3]) + (polyn_br[2] * (dir[3] * dir[3])) + (polyn_br[3] * (dir[3] * dir[3] * dir[3])) + (polyn_br[4] * (dir[3] * dir[3] * dir[3] * dir[3]));// + (polyn_br[5] * (dir[3] * dir[3] * dir[3] * dir[3] * dir[3]));
//			direction[3] -= 25;
//	}
//	else{
//		direction[3] = 0;
//	}
//
//	// FL
//	if(dir[0] > 0){
//			direction[0] = (poly_fl[0] * 1) + (poly_fl[1] * dir[0]) + (poly_fl[2] * (dir[0] * dir[0])) + (poly_fl[3] * (dir[0] * dir[0] * dir[0])) + (poly_fl[4] * (dir[0] * dir[0] * dir[0] * dir[0]));
//	}
//	else if(dir[0] < 0) {
//			direction[0] = (poltransmit_motoryn_fl[0] * 1) + (polyn_fl[1] * dir[0]) + (polyn_fl[2] * (dir[0] * dir[0])) + (polyn_fl[3] * (dir[0] * dir[0] * dir[0])) + (polyn_fl[4] * (dir[0] * dir[0] * dir[0] * dir[0]));
//	}
//	else{
//		direction[0] = 0;
//	}
//
//	//BL
//	if(dir[2] > 0){
//			direction[2] = (poly_bl[0] * 1) + (poly_bl[1] * dir[2]) + (poly_bl[2] * (dir[2] * dir[2])) + (poly_bl[3] * (dir[2] * dir[2] * dir[2])) + (poly_bl[4] * (dir[2] * dir[2] * dir[2] * dir[2]));
//	}
//	else if(dir[2] < 0){
//			direction[2] = (polyn_bl[0] * 1) + (polyn_bl[1] * dir[2]) + (polyn_bl[2] * (dir[2] * dir[2])) + (polyn_bl[3] * (dir[2] * dir[2] * dir[2])) + (polyn_bl[4] * (dir[2] * dir[2] * dir[2] * dir[2]));
//	}
//	else{
//		direction[2] = 0;
//	}
//}

void rpm_publisher(void) {
	rpm_msg.x = rpm[0];
	rpm_msg.y = rpm[1];
	rpm_msg.z = rpm[2];
	rpm_msg.w = rpm[3];
	rpm_init.publish(&rpm_msg);
}
