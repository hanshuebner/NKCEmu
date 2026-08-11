/* bote.c - the bote card: a courier carrying one request out and one reply
 * back, as the hanse board emulates it at ports 58h (data) and 59h (status).
 *
 * The machine writes request bytes to the data port, sends them with GO on
 * the status port, polls BUSY out and reads the reply from the data port
 * while DATA stands.  Here the carry is synchronous: a GO posts the request
 * to the score server over HTTP inside the OUT, so the machine never sees
 * BUSY and a program developed against this emulator still polls it, as the
 * real card does stand busy.
 *
 * NKC_BOTE names the server as host:port (default scores.netmbx.org:8137,
 * the league's own board, as the hanse firmware defaults to) and
 * NKC_BOTE_PATH the path (default /).  A server that cannot be reached
 * leaves the FAILED bit, which is what the far end of a broken network
 * looks like on the real card too.
 */

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "sim.h"
#include "simglb.h"

#define BOTE_REQUEST_MAX 256
#define BOTE_REPLY_MAX 2048

#define BOTE_ST_ROOM 0x01
#define BOTE_ST_DATA 0x02
#define BOTE_ST_FAILED 0x40
#define BOTE_ST_BUSY 0x80

#define BOTE_GO 0x01
#define BOTE_RESET 0x02

static BYTE request[BOTE_REQUEST_MAX];
static unsigned reqlen;
static BYTE reply[BOTE_REPLY_MAX];
static unsigned replylen, replyat;
static int failed;

/* One POST, blocking, bounded by a socket timeout.  Returns the body length
 * landed in reply[], or -1. */
static int carry(void)
{
    const char *spec = getenv("NKC_BOTE");
    const char *path = getenv("NKC_BOTE_PATH");
    char host[128];
    int port = 8137;
    struct addrinfo hints, *found, *ai;
    struct timeval patience = {3, 0};
    char head[512];
    static char raw[BOTE_REPLY_MAX + 4096];
    int fd = -1, rawlen = 0, n;
    char *body;

    snprintf(host, sizeof(host), "%s", spec && *spec ? spec : "scores.netmbx.org");
    {
        char *colon = strrchr(host, ':');
        if (colon) {
            *colon = 0;
            port = atoi(colon + 1);
        }
    }
    if (!path || !*path)
        path = "/";

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    {
        char portname[16];
        snprintf(portname, sizeof(portname), "%d", port);
        if (getaddrinfo(host, portname, &hints, &found) != 0)
            return -1;
    }
    for (ai = found; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
            continue;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &patience, sizeof(patience));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &patience, sizeof(patience));
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
            break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(found);
    if (fd < 0)
        return -1;

    n = snprintf(head, sizeof(head),
                 "POST %s HTTP/1.0\r\nHost: %s\r\nContent-Length: %u\r\n\r\n",
                 path, host, reqlen);
    if (write(fd, head, n) != n || write(fd, request, reqlen) != (int) reqlen) {
        close(fd);
        return -1;
    }
    while (rawlen < (int) sizeof(raw) - 1 &&
           (n = (int) read(fd, raw + rawlen, sizeof(raw) - 1 - rawlen)) > 0)
        rawlen += n;
    close(fd);
    raw[rawlen] = 0;

    if (strncmp(raw, "HTTP/1.", 7) != 0 || !strstr(raw, " 200"))
        return -1;
    body = strstr(raw, "\r\n\r\n");
    if (!body)
        return -1;
    body += 4;
    n = rawlen - (int) (body - raw);
    if (n > BOTE_REPLY_MAX)
        n = BOTE_REPLY_MAX;
    memcpy(reply, body, n);
    return n;
}

BYTE bote_p58_in(void)
{
    if (replyat < replylen)
        return reply[replyat++];
    return 0;
}

void bote_p58_out(BYTE b)
{
    if (reqlen < BOTE_REQUEST_MAX)
        request[reqlen++] = b;
}

BYTE bote_p59_in(void)
{
    BYTE status = 0;

    if (reqlen < BOTE_REQUEST_MAX)
        status |= BOTE_ST_ROOM;
    if (replyat < replylen)
        status |= BOTE_ST_DATA;
    if (failed)
        status |= BOTE_ST_FAILED;
    return status;
}

void bote_p59_out(BYTE b)
{
    if (b == BOTE_RESET) {
        reqlen = 0;
        replylen = replyat = 0;
        failed = 0;
        return;
    }
    if (b != BOTE_GO)
        return;
    failed = 0;
    replylen = replyat = 0;
    {
        int n = carry();
        if (n < 0)
            failed = 1;
        else
            replylen = (unsigned) n;
    }
    reqlen = 0;
}
