/*
 * dvl.c
 *
 *  Created on: May 12, 2025
 *      Author: Martin
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

void delMelding(char *melding) {
	/*Del opp hver melding på ','.
	 * Sett sammen ny kommadelt meldingsstreng med de delene som skal være med
	 * wrz og wrx meldinger sendes ut som vel melding
	 * wrp meldinger sendes ut som pos melding
	 */
	char temp[256];
	char vel[256] = {0};
	char pos[256] = {0};
	uint8_t newline[] = "\n\r";
	uint8_t komma[] = ", ";

	char test1[10];
	char test2[10];
	char test3[10];
//	uint8_t testType1[] = "Kommet ut";
//	uint8_t testType2[] = "Position";
//	uint8_t feil[] = "Feil";
//	uint8_t testMelding[] = "Meldingsoppdeling";


	strncpy(temp, melding, sizeof(temp));
	temp[sizeof(temp) - 1] = '\0';  		// Sikre null-terminering

	HAL_UART_Transmit(&huart2, (uint8_t *)temp, 10, 100);
	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);

	char *token;
	int count = 0;

/*	for(int i=1;i<5;i++) {
		token = strtok(temp, ",");
		HAL_UART_Transmit(&huart2, (uint8_t *)token, 10, 100);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
	}*/

/*	token = strtok(temp, ",");
	strncpy(test1, token, sizeof(token));
	token = strtok(temp, ",");
	strncpy(test2, token, sizeof(token));
	token = strtok(temp, ",");
	strncpy(test2, token, sizeof(token));


	HAL_UART_Transmit(&huart2, (uint8_t *)test1, 10, 100);
	HAL_UART_Transmit(&huart2, (uint8_t *)komma, 2, 100);
	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);*/


	//La det under ligge og test med enklere
	//Sortér etter meldingstype
/*	if (strncmp(temp, "wrx", 3) == 0 || strncmp(temp,"wrz", 3) == 0) {
		// Velocity-melding
		strcpy(vel, "vel,");
		count = 0;
		token = strtok(temp, ",");
		token = strtok(NULL, ",");

		while (token != NULL && count < 5) {
			strcat(vel, token);
			count++;
			if (count < 5) {
				strcat(vel, ",");
			}
			token = strtok(NULL, ",");
		}
		HAL_UART_Transmit(&huart2, (uint8_t *)vel, 50, 100);
		HAL_UART_Transmit(&huart2, newline, 2, 100);
	}
    else if (strncmp(temp, "wrp", 3) == 0) {
        // Position-melding
        strcpy(pos, "pos,");
        count = 0;
        token = strtok(temp, ",");
        token = strtok(NULL, ",");

        while (token != NULL && count < 5) {
            strcat(pos, token);
            count++;
            if (count < 5) {
                strcat(pos, ",");
            }
            token = strtok(NULL, ",");
        }

        HAL_UART_Transmit(&huart2, (uint8_t *)pos, 50, 100);
        HAL_UART_Transmit(&huart2, newline, 2, 100);
    }
    else {
        // Ukjent meldingstype, gjør ingenting (eller skriv en feilmelding hvis du vil)
    }*/


//	HAL_UART_Transmit(&huart2, melding, BUFFER_SIZE, 100);
//	HAL_UART_Transmit(&huart2, newline, 2, 100);

}
