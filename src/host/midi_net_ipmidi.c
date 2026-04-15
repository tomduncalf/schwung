/*
 * midi_net_ipmidi.c - ipMIDI multicast backend
 *
 * Receives raw MIDI over UDP multicast (group 225.0.0.37, port 21928).
 * ipMIDI is a stateless, session-less protocol used by iConnectivity's
 * mioXL, MidiOverLan CP, Tobias Erichsen's rtpMIDI in "ipMIDI mode", and
 * the `sendmidi` CLI tool. Payload is simply concatenated raw MIDI bytes
 * with running status preserved across messages.
 *
 * Phase 1: inbound only.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include "midi_net.h"
#include "midi_net_internal.h"

/* ============================================================================
 * Multicast group join across all interfaces
 * ============================================================================ */

static int join_on_all_interfaces(int sock, const char *group_addr) {
    struct ifaddrs *ifa_list = NULL;
    if (getifaddrs(&ifa_list) != 0) {
        midi_net_log("ipMIDI: getifaddrs failed: %s", strerror(errno));
        return -1;
    }

    int joined = 0;
    struct in_addr group;
    inet_pton(AF_INET, group_addr, &group);

    for (struct ifaddrs *ifa = ifa_list; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        if (!(ifa->ifa_flags & IFF_UP)) continue;
        if (!(ifa->ifa_flags & IFF_RUNNING)) continue;
        if (!(ifa->ifa_flags & IFF_MULTICAST) && !(ifa->ifa_flags & IFF_LOOPBACK)) continue;

        struct sockaddr_in *sin = (struct sockaddr_in *)ifa->ifa_addr;

        struct ip_mreq mreq;
        memset(&mreq, 0, sizeof(mreq));
        mreq.imr_multiaddr = group;
        mreq.imr_interface = sin->sin_addr;

        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       &mreq, sizeof(mreq)) == 0) {
            joined++;
            char ipbuf[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &sin->sin_addr, ipbuf, sizeof(ipbuf));
            midi_net_logd("ipMIDI: joined %s on %s (%s)",
                          group_addr, ifa->ifa_name, ipbuf);
        } else if (errno != EADDRINUSE) {
            midi_net_logd("ipMIDI: join on %s failed: %s",
                          ifa->ifa_name, strerror(errno));
        }
    }
    freeifaddrs(ifa_list);
    return joined;
}

/* ============================================================================
 * Socket lifecycle
 * ============================================================================ */

int midi_net_ipmidi_open(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        midi_net_log("ipMIDI: socket failed: %s", strerror(errno));
        return -1;
    }

    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(MIDI_NET_IPMIDI_PORT);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        midi_net_log("ipMIDI: bind to :%d failed: %s",
                     MIDI_NET_IPMIDI_PORT, strerror(errno));
        close(sock);
        return -1;
    }

    /* Disable loopback so we don't echo our own sends when Phase 4 lands */
    uint8_t loop = 1;  /* keep loopback on for local-machine testing in dev */
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    int joined = join_on_all_interfaces(sock, MIDI_NET_IPMIDI_GROUP);
    if (joined <= 0) {
        midi_net_log("ipMIDI: no multicast-capable interfaces up, will retry");
    }

    g_midi_net.ipmidi_sock = sock;
    g_midi_net.ipmidi_last_status = 0;
    return 0;
}

void midi_net_ipmidi_close(void) {
    if (g_midi_net.ipmidi_sock >= 0) {
        close(g_midi_net.ipmidi_sock);
        g_midi_net.ipmidi_sock = -1;
    }
}

/* ============================================================================
 * Receive and parse
 * ============================================================================ */

void midi_net_ipmidi_handle_rx(int sock) {
    uint8_t buf[1500];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    for (int loop = 0; loop < 16; loop++) {
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0,
                             (struct sockaddr *)&from, &fromlen);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            midi_net_logd("ipMIDI: recvfrom error: %s", strerror(errno));
            break;
        }
        if (n == 0) continue;

        g_midi_net.stats.rx_packets++;

        int msgs = midi_net_parse_raw_stream(buf, (int)n,
                                             &g_midi_net.ipmidi_last_status);
        if (g_midi_net.stats.rx_packets <= 5 || (g_midi_net.stats.rx_packets % 1024) == 0) {
            char src[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &from.sin_addr, src, sizeof(src));
            midi_net_logd("ipMIDI: rx %zd bytes from %s, parsed %d msgs", n, src, msgs);
        }
    }
}
