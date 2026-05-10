#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <math.h>

// Inclusion des headers MAVLink
// Chemin relatif depuis workspace/src -> workspace/libs
#include "../libs/mavlink/common/mavlink.h"

#define TARGET_IP "127.0.0.1"
#define TARGET_PORT 14580
#define BUFFER_LENGTH 2048

int main() {
    int sock;
    struct sockaddr_in targetAddr; 
    struct sockaddr_in locAddr;
    struct sockaddr_in fromAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;

    // 1. Création du Socket UDP
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14540); // Notre port d'écoute (arbitraire)

    if (-1 == bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    } 

    // Configuration de l'adresse cible (PX4 SITL)
    memset(&targetAddr, 0, sizeof(targetAddr));
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_addr.s_addr = inet_addr(TARGET_IP);
    targetAddr.sin_port = htons(TARGET_PORT);

    printf("Attente du Heartbeat du drone...\n");

    // 2. Boucle principale
    // Pour cet exemple simple, on envoie les commandes une fois qu'on a reçu un paquet
    int connection_established = 0;
    
    while(1) {
        memset(buf, 0, BUFFER_LENGTH);
        fromlen = sizeof(fromAddr);
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&fromAddr, &fromlen);
        
        if (recsize > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;

            // Parsing du paquet reçu
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);
                        printf("COMMAND_ACK: command=%u result=%u\n", ack.command, ack.result);
                    }
                    
                    // Si on reçoit un HEARTBEAT, la connexion est établie
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        if (!connection_established) {
                            printf("Drone connecté !\n");
                            connection_established = 1;

                            const uint8_t target_system = msg.sysid;
                            const uint8_t target_component = msg.compid;

                            // ÉTAPE A : ARMEMENT (Allumer les moteurs)
                            // On construit la commande MAV_CMD_COMPONENT_ARM_DISARM
                            mavlink_message_t cmd_msg;
                            mavlink_command_long_t cmd;

                            memset(&cmd, 0, sizeof(cmd));
                            cmd.target_system = target_system;
                            cmd.target_component = target_component;
                            cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
                            cmd.confirmation = 0;
                            cmd.param1 = 1; // 1 = ARM, 0 = DISARM
                            
                            // Encodage du message
                            mavlink_msg_command_long_encode(255, 1, &cmd_msg, &cmd);
                            uint8_t tx_buf[BUFFER_LENGTH];
                            int len = mavlink_msg_to_send_buffer(tx_buf, &cmd_msg);
                            
                            sendto(sock, tx_buf, len, 0, (struct sockaddr*)&targetAddr, sizeof(struct sockaddr_in));
                            printf("Commande ARM envoyée.\n");

                            sleep(2); // Petite pause pour laisser le temps d'armer

                            // ÉTAPE B : DÉCOLLAGE (Takeoff)
                            // MAV_CMD_NAV_TAKEOFF
                            memset(&cmd, 0, sizeof(cmd));
                            cmd.target_system = target_system;
                            cmd.target_component = target_component;
                            cmd.command = MAV_CMD_NAV_TAKEOFF;
                            cmd.param1 = 0; // Pitch min
                            cmd.param2 = 0;
                            cmd.param3 = 0;
                            cmd.param4 = 0; // Yaw
                            // For PX4, use NaN for "use current" rather than 0,0 (which is a real coordinate)
                            cmd.param5 = NAN; // Latitude
                            cmd.param6 = NAN; // Longitude
                            cmd.param7 = 500; // Altitude (10 mètres)
                            
                            mavlink_msg_command_long_encode(255, 1, &cmd_msg, &cmd);
                            len = mavlink_msg_to_send_buffer(tx_buf, &cmd_msg);
                            
                            sendto(sock, tx_buf, len, 0, (struct sockaddr*)&targetAddr, sizeof(struct sockaddr_in));
                            printf("Commande TAKEOFF envoyée (10m).\n");
                        }
                    }
                }
            }
        }
    }
    return 0;
}