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

static int send_command_long(int sock,
                             const struct sockaddr_in *target_addr,
                             uint8_t target_system,
                             uint8_t target_component,
                             uint16_t command,
                             float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    mavlink_message_t cmd_msg;
    mavlink_command_long_t cmd;
    uint8_t tx_buf[BUFFER_LENGTH];

    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = target_system;
    cmd.target_component = target_component;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.param3 = p3;
    cmd.param4 = p4;
    cmd.param5 = p5;
    cmd.param6 = p6;
    cmd.param7 = p7;

    mavlink_msg_command_long_encode(255, 1, &cmd_msg, &cmd);
    int len = mavlink_msg_to_send_buffer(tx_buf, &cmd_msg);
    return (int)sendto(sock, tx_buf, len, 0, (const struct sockaddr *)target_addr, sizeof(*target_addr));
}

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
    int rtl_command_sent = 0;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    
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

                    if (msg.msgid == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
                        mavlink_extended_sys_state_t state;
                        mavlink_msg_extended_sys_state_decode(&msg, &state);

                        if (rtl_command_sent &&
                            state.landed_state == MAV_LANDED_STATE_ON_GROUND) {
                            printf("Retour au sol confirmé (ON_GROUND).\n");
                        }
                    }
                    
                    // Si on reçoit un HEARTBEAT, la connexion est établie
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        if (!connection_established) {
                            printf("Drone connecté !\n");
                            connection_established = 1;

                            target_system = msg.sysid;
                            target_component = msg.compid;

                            // Ask PX4 to stream EXTENDED_SYS_STATE so we can reliably detect ON_GROUND.
                            // MAV_CMD_SET_MESSAGE_INTERVAL: param1=message id, param2=interval (us)
                            send_command_long(sock, &targetAddr, target_system, target_component,
                                              MAV_CMD_SET_MESSAGE_INTERVAL,
                                              (float)MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                                              200000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                            // ÉTAPE A : RETOUR AU POINT DE DÉPART (Return to Launch)
                            send_command_long(sock, &targetAddr, target_system, target_component,
                                              MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                            rtl_command_sent = 1;
                            printf("Commande RTL envoyée. Le drone retourne au point de départ et atterrit...\n");
                        }
                    }
                }
            }
        }
    }
    return 0;
}
