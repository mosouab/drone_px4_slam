#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <math.h>

// Inclusion des headers MAVLink
// Chemin relatif depuis workspace/src -> workspace/libs
#include "../../libs/mavlink/common/mavlink.h"

#define TARGET_IP "127.0.0.1"
#define TARGET_PORT 14580
#define BUFFER_LENGTH 2048

// Structure pour définir un waypoint
typedef struct {
    double lat_deg;
    double lon_deg;
    float alt_m;
} waypoint_t;

static int send_message(int sock, const struct sockaddr_in *target_addr, const mavlink_message_t *msg)
{
    uint8_t tx_buf[BUFFER_LENGTH];
    int len = mavlink_msg_to_send_buffer(tx_buf, msg);
    return (int)sendto(sock, tx_buf, len, 0, (const struct sockaddr *)target_addr, sizeof(*target_addr));
}

static int send_command_long(int sock,
                             const struct sockaddr_in *target_addr,
                             uint8_t target_system,
                             uint8_t target_component,
                             uint16_t command,
                             float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    mavlink_message_t cmd_msg;
    mavlink_command_long_t cmd;

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
    return send_message(sock, target_addr, &cmd_msg);
}

static int send_mission_count(int sock,
                              const struct sockaddr_in *target_addr,
                              uint8_t target_system,
                              uint8_t target_component,
                              uint16_t count)
{
    mavlink_message_t msg;
    mavlink_mission_count_t mc;
    
    memset(&mc, 0, sizeof(mc));
    mc.target_system = target_system;
    mc.target_component = target_component;
    mc.count = count;
    mc.mission_type = MAV_MISSION_TYPE_MISSION;

    mavlink_msg_mission_count_encode(255, 1, &msg, &mc);
    return send_message(sock, target_addr, &msg);
}

static int send_mission_item_int(int sock,
                                 const struct sockaddr_in *target_addr,
                                 uint8_t target_system,
                                 uint8_t target_component,
                                 uint16_t seq,
                                 uint16_t command,
                                 uint8_t frame,
                                 float p1, float p2, float p3, float p4,
                                 int32_t x, int32_t y, float z,
                                 uint8_t current)
{
    mavlink_message_t msg;
    mavlink_mission_item_int_t item;
    
    memset(&item, 0, sizeof(item));
    item.target_system = target_system;
    item.target_component = target_component;
    item.seq = seq;
    item.frame = frame;
    item.command = command;
    item.current = current;
    item.autocontinue = 1;
    item.param1 = p1;
    item.param2 = p2;
    item.param3 = p3;
    item.param4 = p4;
    item.x = x;
    item.y = y;
    item.z = z;
    item.mission_type = MAV_MISSION_TYPE_MISSION;

    mavlink_msg_mission_item_int_encode(255, 1, &msg, &item);
    return send_message(sock, target_addr, &msg);
}

int main() {
    int sock;
    struct sockaddr_in targetAddr; 
    struct sockaddr_in locAddr;
    struct sockaddr_in fromAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;

    const waypoint_t waypoints[] = {
        {47.3975000, 8.5456000, 500.0f},  // Point bas du cœur
        {47.3976500, 8.5454000, 500.0f},  // Côté gauche montant
        {47.3978500, 8.5452500, 500.0f},  // Lobe gauche - bas
        {47.3980500, 8.5453000, 500.0f},  // Lobe gauche - milieu
        {47.3982000, 8.5454500, 500.0f},  // Lobe gauche - sommet
        {47.3982500, 8.5456000, 500.0f},  // Centre haut gauche
        {47.3982000, 8.5457500, 500.0f},  // Centre haut
        {47.3982500, 8.5459000, 500.0f},  // Centre haut droit
        {47.3982000, 8.5460500, 500.0f},  // Lobe droit - sommet
        {47.3980500, 8.5462000, 500.0f},  // Lobe droit - milieu
        {47.3978500, 8.5462500, 500.0f},  // Lobe droit - bas
        {47.3976500, 8.5461000, 500.0f},  // Côté droit descendant
        {47.3975000, 8.5459000, 500.0f},  // Retour vers le point bas
        {47.3975000, 8.5456000, 500.0f},  // Point bas du cœur (fermeture)
    };
    const int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

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
    int connection_established = 0;
    int mission_upload_started = 0;
    int mission_started = 0;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    
    // Mission : TAKEOFF + waypoints + LAND
    const int mission_count = 1 + num_waypoints + 1;
    
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
                            target_system = msg.sysid;
                            target_component = msg.compid;
                        }

                        // Démarrer l'upload de mission après connexion
                        if (connection_established && !mission_upload_started) {
                            printf("Upload de la mission (%d items)...\n", mission_count);
                            send_mission_count(sock, &targetAddr, target_system, target_component, mission_count);
                            mission_upload_started = 1;
                        }
                    }

                    // Le drone demande un item de mission
                    if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
                        mavlink_mission_request_int_t req;
                        mavlink_msg_mission_request_int_decode(&msg, &req);

                        if (req.mission_type != MAV_MISSION_TYPE_MISSION) {
                            continue;
                        }

                        int32_t lat, lon;

                        if (req.seq == 0) {
                            // Item 0 : TAKEOFF
                            lat = (int32_t)(waypoints[0].lat_deg * 1e7);
                            lon = (int32_t)(waypoints[0].lon_deg * 1e7);
                            send_mission_item_int(sock, &targetAddr, target_system, target_component,
                                                  req.seq, MAV_CMD_NAV_TAKEOFF,
                                                  MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0.0f, 0.0f, 0.0f, NAN,
                                                  lat, lon, 500.0f, 1);
                            printf("Item %d/%d envoyé: TAKEOFF\n", req.seq + 1, mission_count);
                        } 
                        else if (req.seq <= num_waypoints) {
                            // Items 1 à num_waypoints : WAYPOINT
                            int wp_idx = req.seq - 1;
                            lat = (int32_t)(waypoints[wp_idx].lat_deg * 1e7);
                            lon = (int32_t)(waypoints[wp_idx].lon_deg * 1e7);
                            send_mission_item_int(sock, &targetAddr, target_system, target_component,
                                                  req.seq, MAV_CMD_NAV_WAYPOINT,
                                                  MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0.0f, 2.0f, 0.0f, NAN,
                                                  lat, lon, waypoints[wp_idx].alt_m, 0);
                            printf("Item %d/%d envoyé: WAYPOINT %d\n", req.seq + 1, mission_count, wp_idx + 1);
                        } 
                        else {
                            // Dernier item : LAND
                            int last_idx = num_waypoints - 1;
                            lat = (int32_t)(waypoints[last_idx].lat_deg * 1e7);
                            lon = (int32_t)(waypoints[last_idx].lon_deg * 1e7);
                            send_mission_item_int(sock, &targetAddr, target_system, target_component,
                                                  req.seq, MAV_CMD_NAV_LAND,
                                                  MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                  0.0f, 0.0f, 0.0f, NAN,
                                                  lat, lon, 0.0f, 0);
                            printf("Item %d/%d envoyé: LAND\n", req.seq + 1, mission_count);
                        }
                    }

                    // Confirmation que la mission est acceptée
                    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);
                        
                        if (ack.mission_type == MAV_MISSION_TYPE_MISSION && !mission_started) {
                            if (ack.type == MAV_MISSION_ACCEPTED) {
                                printf("Mission acceptée ! Démarrage...\n");
                                
                                // ÉTAPE A : ARMEMENT
                                send_command_long(sock, &targetAddr, target_system, target_component,
                                                  MAV_CMD_COMPONENT_ARM_DISARM,
                                                  1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                                printf("Commande ARM envoyée.\n");
                                sleep(2);

                                // ÉTAPE B : DÉMARRAGE DE LA MISSION
                                send_command_long(sock, &targetAddr, target_system, target_component,
                                                  MAV_CMD_MISSION_START,
                                                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                                printf("Mission démarrée ! Le drone va suivre les waypoints en forme de cœur.\n");
                                mission_started = 1;
                            } else {
                                printf("Erreur mission: type=%u\n", ack.type);
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}
