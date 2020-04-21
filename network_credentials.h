/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef NETWORK_CREDENTIALS_H_
#define NETWORK_CREDENTIALS_H_

/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_KEY to match your Wi-Fi network
 * Credentials.
 */
#ifdef __USE_AP__
	#define WIFI_SSID                         "AP"
	#define WIFI_PASSWORD                     "12345678"
#else
	#define WIFI_SSID                         "DIGI_4834e8"
	#define WIFI_PASSWORD                     "f988b7cb"
#endif

#endif /* NETWORK_CREDENTIALS_H_ */
