/*
 * midi_net.h - MIDI over WiFi (network MIDI) subsystem
 *
 * Receives MIDI over WiFi via multiple backends (ipMIDI, AppleMIDI/RTP-MIDI)
 * and injects the resulting USB-MIDI packets into Move's MIDI_IN stream via
 * the existing /schwung-midi-inject shared memory buffer.
 *
 * Runs a single background pthread owning all UDP sockets. No modifications
 * to Move's core MIDI routing: network MIDI flows through the normal inject
 * drain in shadow_midi.c and appears as cable-0 internal hardware MIDI.
 *
 * Phase 1: ipMIDI inbound (multicast, no session)
 * Phase 2: AppleMIDI session protocol inbound
 * Phase 3: mDNS/Bonjour advertisement
 * Phase 4: Outbound publishing
 */

#ifndef MIDI_NET_H
#define MIDI_NET_H

#include <stdint.h>
#include "shadow_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ============================================================================ */

/* ipMIDI (iConnectivity / MidiOverLan CP) */
#define MIDI_NET_IPMIDI_GROUP    "225.0.0.37"
#define MIDI_NET_IPMIDI_PORT     21928

/* AppleMIDI / RTP-MIDI */
#define MIDI_NET_APPLE_CTRL_PORT 5004
#define MIDI_NET_APPLE_EVT_PORT  5005

/* mDNS */
#define MIDI_NET_MDNS_GROUP      "224.0.0.251"
#define MIDI_NET_MDNS_PORT       5353

/* Feature flag file (Phase 1 — promoted to Global Settings in Phase 5) */
#define MIDI_NET_FLAG_FILE       "/data/UserData/schwung/midi_net_on"

/* Backend enable flags (bitmask) */
#define MIDI_NET_BACKEND_IPMIDI     0x01
#define MIDI_NET_BACKEND_APPLEMIDI  0x02
#define MIDI_NET_BACKEND_MDNS       0x04
#define MIDI_NET_BACKEND_OUTBOUND   0x08
#define MIDI_NET_BACKEND_ALL        0x0F

/* Advertised session name (mDNS) */
#define MIDI_NET_SESSION_NAME    "Schwung-Move"

/* ============================================================================
 * Initialization
 * ============================================================================ */

/*
 * Provide the inject SHM pointer address and logging function.
 * The pointer is dereferenced each packet so the caller's SHM can become
 * available after init (e.g. if SHM map fails briefly on startup).
 * log_fn is optional — if non-NULL, it will be called with short human
 * messages. Set to NULL to use the unified_log subsystem instead.
 */
void midi_net_init(shadow_midi_inject_t **inject_shm_ptr,
                   void (*log_fn)(const char *msg));

/*
 * Select which backends to run.
 * Default is MIDI_NET_BACKEND_ALL. Call before midi_net_start() to change.
 * Changing after start is not supported — call stop, set, start.
 */
void midi_net_set_backends(uint32_t backend_flags);

/* ============================================================================
 * Lifecycle
 * ============================================================================ */

/*
 * Start the background network thread (idempotent). Returns 0 on success,
 * -1 if thread could not be created. Does NOT check the feature flag file;
 * the caller is expected to gate this call appropriately.
 */
int midi_net_start(void);

/*
 * Stop the background network thread (idempotent). Blocks up to ~500ms
 * waiting for clean shutdown.
 */
void midi_net_stop(void);

/* Returns 1 if the network thread is currently running. */
int midi_net_is_running(void);

/* ============================================================================
 * Feature flag
 * ============================================================================ */

/*
 * Check the flag file on disk. Cached for ~1 second to avoid hammering
 * the filesystem. Call from the shim's ioctl loop to react to toggles.
 */
int midi_net_flag_enabled(void);

/* ============================================================================
 * Outbound (Phase 4)
 * ============================================================================ */

/*
 * Publish a 4-byte USB-MIDI packet to network peers.
 * Called from shadow_midi.c MIDI_OUT drain path.
 * No-op if Phase 4 outbound backend is not enabled.
 * Safe to call from any thread.
 */
void midi_net_publish(const uint8_t *pkt4);

/* ============================================================================
 * Stats (diagnostics, read-only)
 * ============================================================================ */

typedef struct midi_net_stats_t {
    uint64_t rx_packets;        /* Raw UDP packets received (all backends) */
    uint64_t rx_midi_messages;  /* Parsed MIDI messages delivered to inject SHM */
    uint64_t tx_packets;        /* Outbound packets sent (Phase 4) */
    uint64_t drops_buffer_full; /* Packets dropped because inject SHM was full */
    uint64_t drops_parse_err;   /* Packets dropped because of parse errors */
    uint32_t applemidi_peers;   /* Currently-connected AppleMIDI sessions */
    uint32_t mdns_queries;      /* mDNS queries seen */
    uint32_t mdns_responses;    /* mDNS responses sent */
} midi_net_stats_t;

void midi_net_get_stats(midi_net_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MIDI_NET_H */
