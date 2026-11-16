// test_diagnostics.cpp  v2026.11.16.1  (16 Nov 2026 19:45)
// Host-only test (never copy into the component folder): receive-side diagnostics.
//   1. Framer bad-frame capture (idle drop, resync skip, overflow) without changing framing.
//   2. GatewayResponder no-reply reason + first unseen register; reply bytes unchanged.
//   3. Byte timestamps: bytes stamped late (stale) get a partial frame dropped; bytes stamped
//      when read keep it, so the full frame decodes.
// Build: g++ -std=c++17 -Wall -Wextra -I../components/thermia_gateway test_diagnostics.cpp -o t && ./t
#include <cstdio>
#include <cstring>
#include <vector>
#include "gateway_responder.h"

using modbus_rtu::Framer;
using cm930::GatewayResponder;

static int fails = 0;
static void check(const char *what, bool ok) { if (!ok) fails++; printf("%-4s %s\n", ok ? "ok" : "FAIL", what); }

static std::vector<uint8_t> with_crc(std::vector<uint8_t> f) {
  uint16_t c = modbus_rtu::crc16(f.data(), f.size());
  f.push_back(c & 0xFF);
  f.push_back(c >> 8);
  return f;
}

// The exact mailbox reply the ESP sends today (from Alan's log, 16 Nov 2026 17:26:51).
static const uint8_t MBX_REPLY[17] = {0x0F, 0x03, 0x0C, 0, 0, 0, 0, 0, 0, 0, 0, 0x07, 0x7F, 0x00, 0x06, 0xAD, 0xDA};

int main() {
  const uint8_t *d;
  size_t n;

  // ---- 1. Framer bad-frame capture ----
  {
    Framer f(3000, 30000);
    const uint8_t junk[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    for (int i = 0; i < 5; i++) f.push(junk[i], 1000 + i * 1000);
    check("junk: no frame before drop time", !f.poll(5000 + 10000, &d, &n));
    check("junk: nothing captured before drop time", !f.bad_pending());
    check("junk: no frame at drop time", !f.poll(5000 + 40000, &d, &n));
    check("junk: captured as IDLE_DROP", f.bad_pending() && f.last_bad_reason() == Framer::BadReason::IDLE_DROP);
    check("junk: 5 bytes captured, identical", f.last_bad_len() == 5 && f.last_bad_total() == 5 &&
                                                   memcmp(f.last_bad_data(), junk, 5) == 0);
    check("junk: bad() counted once", f.bad() == 1);
    f.clear_bad_pending();
    check("clear_bad_pending() clears the flag", !f.bad_pending());
  }
  {
    Framer f(3000, 30000);
    const uint8_t lead[3] = {0xAA, 0xBB, 0xCC};
    uint32_t t = 0;
    for (uint8_t b : lead) f.push(b, t += 1000);
    for (uint8_t b : MBX_REPLY) f.push(b, t += 1000);
    bool got = f.poll(t + 4000, &d, &n);
    check("resync: valid frame still emitted after 3 leading bytes",
          got && n == sizeof(MBX_REPLY) && memcmp(d, MBX_REPLY, n) == 0);
    check("resync: skipped bytes captured as RESYNC_SKIP",
          f.bad_pending() && f.last_bad_reason() == Framer::BadReason::RESYNC_SKIP && f.last_bad_len() == 3 &&
              memcmp(f.last_bad_data(), lead, 3) == 0);
  }
  {
    Framer f(3000, 30000);
    for (size_t i = 0; i < Framer::MAX + 1; i++) f.push((uint8_t) i, (uint32_t) i * 10);
    check("overflow: captured as OVERFLOW", f.bad_pending() && f.last_bad_reason() == Framer::BadReason::OVERFLOW);
    check("overflow: first 64 of 260 bytes kept", f.last_bad_len() == Framer::BAD_CAP && f.last_bad_total() == Framer::MAX &&
                                                     f.last_bad_data()[0] == 0 && f.last_bad_data()[63] == 63);
  }
  {
    Framer f(3000, 30000);
    uint32_t t = 0;
    for (uint8_t b : MBX_REPLY) f.push(b, t += 1000);
    check("clean frame: decoded", f.poll(t + 4000, &d, &n) && n == sizeof(MBX_REPLY));
    check("clean frame: nothing captured", !f.bad_pending() && f.bad() == 0);
  }

  // ---- 2. Responder no-reply reasons ----
  {
    GatewayResponder r;
    // Heat pump writes 2000 x4 (FC10 to 0x0F)
    auto w = with_crc({0x0F, 0x10, 0x07, 0xD0, 0x00, 0x04, 8, 0, 1, 0, 2, 0, 3, 0, 4});
    r.observe(w.data(), w.size());
    auto rd4 = with_crc({0x0F, 0x03, 0x07, 0xD0, 0x00, 0x04});
    auto rd6 = with_crc({0x0F, 0x03, 0x07, 0xD0, 0x00, 0x06});
    auto rep4 = r.build_reply(rd4.data(), rd4.size());
    check("read of seen regs: reply built, reason NONE",
          rep4.has_reply() && rep4.why_none == GatewayResponder::NoReply::NONE);
    auto rep6 = r.build_reply(rd6.data(), rd6.size());
    check("read past seen regs: no reply, UNSEEN_REG", !rep6.has_reply() &&
                                                           rep6.kind == GatewayResponder::Kind::SUPPRESSED_UNSEEN &&
                                                           rep6.why_none == GatewayResponder::NoReply::UNSEEN_REG);
    check("first unseen register is 2004", rep6.first_unseen == 2004);
    auto rd0 = with_crc({0x0F, 0x03, 0x07, 0xD0, 0x00, 0x00});
    auto rep0 = r.build_reply(rd0.data(), rd0.size());
    check("qty 0 read: BAD_REQUEST", !rep0.has_reply() && rep0.why_none == GatewayResponder::NoReply::BAD_REQUEST);
    auto fc05 = with_crc({0x0F, 0x05, 0x00, 0x01, 0xFF, 0x00});
    auto rep5 = r.build_reply(fc05.data(), fc05.size());
    check("FC05: UNSUPPORTED_FC", !rep5.has_reply() && rep5.why_none == GatewayResponder::NoReply::UNSUPPORTED_FC);
    auto other = with_crc({0x02, 0x03, 0x07, 0xD0, 0x00, 0x04});
    auto repo = r.build_reply(other.data(), other.size());
    check("other address: NOT_FOR_US", !repo.has_reply() && repo.why_none == GatewayResponder::NoReply::NOT_FOR_US);
    auto ack8 = with_crc({0x0F, 0x10, 0x07, 0xD0, 0x00, 0x04});
    auto repa = r.build_reply(ack8.data(), ack8.size());
    check("8-byte FC10 (an ack, not a request): BAD_REQUEST",
          !repa.has_reply() && repa.why_none == GatewayResponder::NoReply::BAD_REQUEST);
    // Reply bytes must be exactly what the ESP sends today.
    auto mbx = with_crc({0x0F, 0x03, 0x07, 0x08, 0x00, 0x06});
    auto repm = r.build_reply(mbx.data(), mbx.size());
    check("mailbox reply bytes unchanged (0F 03 0C ... 07 7F 00 06 AD DA)",
          repm.len == sizeof(MBX_REPLY) && memcmp(repm.bytes, MBX_REPLY, repm.len) == 0 && repm.was_mailbox);
    auto repw = r.build_reply(w.data(), w.size());
    auto ackx = with_crc({0x0F, 0x10, 0x07, 0xD0, 0x00, 0x04});
    check("FC10 ack bytes unchanged", repw.len == 8 && memcmp(repw.bytes, ackx.data(), 8) == 0 &&
                                          repw.why_none == GatewayResponder::NoReply::NONE);
  }

  // ---- 3. Byte timestamps ----
  // A 17-byte frame arrives in two parts; the loop is busy ~40 ms between reading them.
  {
    // OLD behaviour: the first part was read during a pass whose `now` was taken BEFORE ~40 ms of
    // other work, so its bytes carry that stale time; poll() then sees >= 30 ms idle and drops it.
    Framer f(3000, 30000);
    const uint32_t loop_top = 100000;  // micros() at the top of loop()
    const uint32_t read_at = loop_top + 40000;
    for (int i = 0; i < 8; i++) f.push(MBX_REPLY[i], loop_top);  // all stamped with the stale time
    bool got = f.poll(read_at + 50, &d, &n);
    check("stale stamps: first part dropped mid-frame (the bug)",
          !got && f.bad_pending() && f.last_bad_reason() == Framer::BadReason::IDLE_DROP && f.last_bad_total() == 8);
    for (int i = 8; i < 17; i++) f.push(MBX_REPLY[i], read_at + 1000 + i * 1040);
    got = f.poll(read_at + 1000 + 17 * 1040 + 4000, &d, &n);
    check("stale stamps: rest never decodes as a frame", !got);
  }
  {
    // NEW behaviour: each byte stamped with micros() when it is read.
    Framer f(3000, 30000);
    const uint32_t read_at = 140000;
    for (int i = 0; i < 8; i++) f.push(MBX_REPLY[i], read_at + i * 5);  // read back-to-back from the FIFO
    bool got = f.poll(read_at + 50, &d, &n);
    check("fresh stamps: partial frame kept (no drop, nothing captured)", !got && !f.bad_pending());
    uint32_t t = read_at + 40;
    for (int i = 8; i < 17; i++) f.push(MBX_REPLY[i], t += 1040);  // rest arrives at 9600 baud
    got = f.poll(t + 4000, &d, &n);
    check("fresh stamps: whole frame decodes", got && n == 17 && memcmp(d, MBX_REPLY, 17) == 0);
  }

  printf("%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
