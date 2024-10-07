#ifndef MODES_H
#define MODES_H

/**
 * The different modes the ekg can be in
*/
enum class Modes {
	/*
	 * Live-mode, the ekg sends realtime data to the app 
	 */
	LIVE,
	/**
	* 24H-Mode, the ekg collects data from the user and stores it onto the sd-card
	*/
	EKG_24H,

};

#endif 


