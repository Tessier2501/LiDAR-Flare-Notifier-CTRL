#include <stdio.h>
#include <string.h>

#include "ptgl_cmd.h"
#include "ptgl_parser.h"

#define FRAME_LEN       PTGL_FRAME_LEN
#define QUEUE_CAPACITY  PTGL_PARSER_QUEUE_CAPACITY
#define TEST_TICK       1234U

static unsigned failures;

static void check(int cond, const char *name)
{
  if (cond == 0) {
    failures++;
    printf("FAIL %s\n", name);
  }
}

static void build_reply(uint8_t frame[FRAME_LEN], uint8_t func, uint8_t sta,
                        uint8_t d0, uint8_t d1, uint8_t d2)
{
  frame[0] = PTGL_HEADER0;
  frame[1] = PTGL_HEADER1;
  frame[2] = func;
  frame[3] = sta;
  frame[4] = d0;
  frame[5] = d1;
  frame[6] = d2;
  frame[7] = ptgl_checksum_reply(frame);
}

static unsigned count_frames(ptgl_parser_t *parser)
{
  unsigned count = 0U;
  ptgl_frame_t frame;

  while (ptgl_parser_pop(parser, &frame, NULL)) {
    count++;
  }
  return count;
}

static void test_command_frames(void)
{
  static const uint8_t single[FRAME_LEN] = {0x55, 0xAA, 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0x84};
  static const uint8_t stream[FRAME_LEN] = {0x55, 0xAA, 0x89, 0xFF, 0xFF, 0xFF, 0xFF, 0x85};
  static const uint8_t stop[FRAME_LEN] = {0x55, 0xAA, 0x8E, 0xFF, 0xFF, 0xFF, 0xFF, 0x8A};
  uint8_t frame[FRAME_LEN];

  ptgl_cmd_single(frame);
  check(memcmp(frame, single, FRAME_LEN) == 0, "cmd_single");

  ptgl_cmd_stream(frame);
  check(memcmp(frame, stream, FRAME_LEN) == 0, "cmd_stream");

  ptgl_cmd_stop(frame);
  check(memcmp(frame, stop, FRAME_LEN) == 0, "cmd_stop");
}

static void test_measure_reply(void)
{
  static const uint8_t raw[FRAME_LEN] = {0x55, 0xAA, 0x88, 0x01, 0x01, 0x02, 0x03, 0x8E};
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(ptgl_parser_pop(&parser, &frame, NULL), "measure_pop");
  check(frame.func == PTGL_FUNC_SINGLE, "measure_func");
  check(frame.sta == PTGL_STA_OK, "measure_sta");
  check(frame.distance_mm == 66051U, "measure_distance");
  check(!ptgl_parser_pop(&parser, &frame, NULL), "measure_single_frame");
  check(parser.frames_ok == 1U, "measure_frames_ok");
  check(parser.checksum_err == 0U, "measure_checksum_err");
}

static void test_measure_failure(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  build_reply(raw, PTGL_FUNC_STREAM, 0x00U, 0x00, 0x00, 0x00);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(ptgl_parser_pop(&parser, &frame, NULL), "sta_fail_pop");
  check(frame.func == PTGL_FUNC_STREAM, "sta_fail_func");
  check(frame.sta == 0x00U, "sta_fail_sta");
  check(frame.distance_mm == 0U, "sta_fail_distance_zero");
}

static void test_self_test_frames(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  build_reply(raw, PTGL_FUNC_SELF_TEST, PTGL_STA_OK, 0x00, 0x00, 0x00);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(ptgl_parser_pop(&parser, &frame, NULL), "selftest_ok_pop");
  check(frame.func == PTGL_FUNC_SELF_TEST, "selftest_ok_func");

  build_reply(raw, PTGL_FUNC_SELF_TEST, 0x00U, 0x00, 0x00, 0x05);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(ptgl_parser_pop(&parser, &frame, NULL), "selftest_fail_pop");
  check(frame.sta == 0x00U, "selftest_fail_sta");
}

static void test_distance_bounds(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  build_reply(raw, PTGL_FUNC_SINGLE, PTGL_STA_OK, 0x00, 0x00, 0x00);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);
  check(ptgl_parser_pop(&parser, &frame, NULL), "distance_zero_pop");
  check(frame.distance_mm == 0U, "distance_zero");

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0xFF, 0xFF, 0xFF);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);
  check(ptgl_parser_pop(&parser, &frame, NULL), "distance_max_pop");
  check(frame.distance_mm == 0xFFFFFFU, "distance_max");
}

static void test_stream_split(void)
{
  uint8_t stream[3U * FRAME_LEN];
  ptgl_parser_t parser;

  build_reply(&stream[0], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x05);
  build_reply(&stream[FRAME_LEN], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x01, 0x00);
  build_reply(&stream[2U * FRAME_LEN], PTGL_FUNC_SELF_TEST, PTGL_STA_OK, 0x00, 0x00, 0x00);

  for (size_t split = 0U; split <= sizeof(stream); split++) {
    ptgl_parser_init(&parser);
    ptgl_parser_feed(&parser, stream, split, TEST_TICK);
    ptgl_parser_feed(&parser, stream + split, sizeof(stream) - split, TEST_TICK);

    check(count_frames(&parser) == 3U, "split_count");
    check(parser.frames_ok == 3U, "split_frames_ok");
    check(parser.checksum_err == 0U, "split_checksum_err");
    check(parser.func_err == 0U, "split_func_err");
  }

  ptgl_parser_init(&parser);
  for (size_t i = 0U; i < sizeof(stream); i++) {
    ptgl_parser_feed(&parser, &stream[i], 1U, TEST_TICK);
  }

  check(count_frames(&parser) == 3U, "bytewise_count");
  check(parser.frames_ok == 3U, "bytewise_frames_ok");
}

static void test_checksum_error_recovery(void)
{
  uint8_t stream[2U * FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  build_reply(&stream[0], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x0A);
  stream[FRAME_LEN - 1U] ^= 0xFFU;
  build_reply(&stream[FRAME_LEN], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x0B);

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, stream, sizeof(stream), TEST_TICK);

  check(parser.checksum_err >= 1U, "checksum_error_counted");
  check(parser.frames_ok == 1U, "checksum_recovery_parsed");
  check(ptgl_parser_pop(&parser, &frame, NULL), "checksum_recovery_pop");
  check(frame.distance_mm == 11U, "checksum_recovery_distance");
}

static void test_unknown_func(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_parser_t parser;

  build_reply(raw, 0x77U, PTGL_STA_OK, 0x00, 0x00, 0x00);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(parser.func_err == 1U, "unknown_func_counted");
  check(parser.frames_ok == 0U, "unknown_func_not_emitted");
  check(parser.checksum_err == 0U, "unknown_func_checksum_clean");
}

static void test_payload_contains_header(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x55U, 0xAAU, 0x03U);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);

  check(parser.frames_ok == 1U, "payload_header_frames_ok");
  check(ptgl_parser_pop(&parser, &frame, NULL), "payload_header_pop");
  check(frame.distance_mm == 0x55AA03U, "payload_header_distance");
  check(!ptgl_parser_pop(&parser, &frame, NULL), "payload_header_no_extra");
}

static void test_noise_resync(void)
{
  static const uint8_t raw[] = {0x00, 0x13, 0x55, 0x55, 0xAA,
                                0x88, 0x01, 0x01, 0x02, 0x03, 0x8E};
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, sizeof(raw), TEST_TICK);

  check(parser.frames_ok == 1U, "noise_frames_ok");
  check(ptgl_parser_pop(&parser, &frame, NULL), "noise_pop");
  check(frame.distance_mm == 66051U, "noise_distance");
}

static void test_timestamp_capture(void)
{
  uint8_t raw[FRAME_LEN];
  uint8_t stream[2U * FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;
  uint32_t t_ms = 0U;

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x2A);

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, raw, 3U, 100U);
  check(!ptgl_parser_pop(&parser, &frame, &t_ms), "timestamp_no_early_frame");
  ptgl_parser_feed(&parser, raw + 3U, FRAME_LEN - 3U, 120U);
  check(ptgl_parser_pop(&parser, &frame, &t_ms), "timestamp_split_pop");
  check(t_ms == 120U, "timestamp_split_completion");

  build_reply(&stream[0], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x01);
  build_reply(&stream[FRAME_LEN], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, 0x02);
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, stream, sizeof(stream), 1000U);
  check(ptgl_parser_pop(&parser, &frame, &t_ms), "timestamp_batch_pop_first");
  check(t_ms == 1000U, "timestamp_batch_first");
  check(ptgl_parser_pop(&parser, &frame, &t_ms), "timestamp_batch_pop_second");
  check(t_ms == 1000U, "timestamp_batch_second");
}

static void test_queue_overflow(void)
{
  const unsigned frame_count = PTGL_PARSER_QUEUE_CAPACITY + 5U;
  const unsigned first_retained = frame_count - QUEUE_CAPACITY;
  ptgl_parser_t parser;
  uint8_t raw[FRAME_LEN];
  ptgl_frame_t frame;

  ptgl_parser_init(&parser);
  for (unsigned i = 0U; i < frame_count; i++) {
    build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, (uint8_t)i);
    ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);
  }

  check(parser.frames_ok == frame_count, "overflow_frames_ok");
  check(parser.overflow == (uint32_t)(frame_count - QUEUE_CAPACITY), "overflow_count");
  check(ptgl_parser_pop(&parser, &frame, NULL), "overflow_first_pop");
  check(frame.distance_mm == first_retained, "overflow_first_retained");
  check(count_frames(&parser) == QUEUE_CAPACITY - 1U, "overflow_popped");
}

static void test_queue_exact_capacity(void)
{
  uint8_t stream[9U * FRAME_LEN];
  uint8_t extra[FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;

  for (unsigned i = 0U; i < 9U; i++) {
    build_reply(&stream[i * FRAME_LEN], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, (uint8_t)i);
  }

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, stream, PTGL_PARSER_QUEUE_CAPACITY * FRAME_LEN, TEST_TICK);
  check(parser.used == PTGL_PARSER_QUEUE_CAPACITY, "batch_used_full");
  check(parser.overflow == 0U, "batch_no_overflow");
  check(count_frames(&parser) == PTGL_PARSER_QUEUE_CAPACITY, "batch_pop_all");

  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, stream, PTGL_PARSER_QUEUE_CAPACITY * FRAME_LEN, TEST_TICK);
  build_reply(extra, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 8U);
  ptgl_parser_feed(&parser, extra, FRAME_LEN, TEST_TICK);
  check(parser.overflow == 1U, "batch_ninth_overflow");
  check(count_frames(&parser) == PTGL_PARSER_QUEUE_CAPACITY, "batch_ninth_retained");
  ptgl_parser_init(&parser);
  ptgl_parser_feed(&parser, stream, 9U * FRAME_LEN, TEST_TICK);
  check(parser.overflow == 1U, "batch_nine_overflow");
  check(ptgl_parser_pop(&parser, &frame, NULL), "batch_newest_pop");
  check(frame.distance_mm == 1U, "batch_drop_oldest");
  check(count_frames(&parser) == PTGL_PARSER_QUEUE_CAPACITY - 1U, "batch_remaining");
}
static void test_stream_100(void)
{
  ptgl_parser_t parser;
  uint8_t raw[FRAME_LEN];
  unsigned parsed = 0U;

  ptgl_parser_init(&parser);
  for (unsigned i = 0U; i < 100U; i++) {
    build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00, 0x00, (uint8_t)i);
    ptgl_parser_feed(&parser, raw, FRAME_LEN, TEST_TICK);
    parsed += count_frames(&parser);
  }

  check(parsed == 100U, "stream_100_count");
  check(parser.frames_ok == 100U, "stream_100_frames_ok");
  check(parser.checksum_err == 0U, "stream_100_checksum_err");
  check(parser.func_err == 0U, "stream_100_func_err");
  check(parser.overflow == 0U, "stream_100_overflow");
}

#define REPLAY_BUF_LEN 20000U

static void test_replay(const char *data_path, const char *expect_path)
{
  static uint8_t data[REPLAY_BUF_LEN];
  FILE *expect;
  FILE *file;
  ptgl_parser_t parser;
  ptgl_frame_t frame;
  size_t size;
  unsigned count = 0U;
  unsigned lines = 0U;
  unsigned mismatches = 0U;
  int read_ok = 1;
  unsigned long expect_distance;
  unsigned expect_sta;

  file = fopen(data_path, "rb");
  check(file != NULL, "replay_open");
  if (file == NULL) {
    return;
  }
  size = fread(data, 1U, sizeof(data), file);
  fclose(file);
  check(size > 0U && size < sizeof(data), "replay_size");

  expect = fopen(expect_path, "r");
  check(expect != NULL, "replay_expect_open");
  if (expect == NULL) {
    return;
  }

  ptgl_parser_init(&parser);
  for (size_t i = 0U; i < size; i++) {
    ptgl_parser_feed(&parser, &data[i], 1U, TEST_TICK);
    while (ptgl_parser_pop(&parser, &frame, NULL)) {
      if (fscanf(expect, "%lu %u", &expect_distance, &expect_sta) != 2) {
        read_ok = 0;
        break;
      }
      lines++;
      if ((unsigned long)frame.distance_mm != expect_distance ||
          (unsigned)frame.sta != expect_sta) {
        mismatches++;
      }
      count++;
    }
    if (read_ok == 0) {
      break;
    }
  }

  check(read_ok != 0, "replay_expect_read");
  check(mismatches == 0U, "replay_values");
  check(count == lines, "replay_line_count");
  check(count > 0U, "replay_frame_count");
  check(parser.frames_ok == count, "replay_frames_ok");
  check(fscanf(expect, "%lu %u", &expect_distance, &expect_sta) == EOF, "replay_expect_exhausted");

  fclose(expect);
}

int main(int argc, char **argv)
{
  test_command_frames();
  test_measure_reply();
  test_measure_failure();
  test_self_test_frames();
  test_distance_bounds();
  test_stream_split();
  test_checksum_error_recovery();
  test_unknown_func();
  test_payload_contains_header();
  test_noise_resync();
  test_timestamp_capture();
  test_queue_overflow();
  test_queue_exact_capacity();
  test_stream_100();

  if (argc == 3) {
    test_replay(argv[1], argv[2]);
  } else {
    check(argc == 1, "replay_arguments");
  }

  printf("tests_failures=%u\n", failures);
  return failures == 0U ? 0 : 1;
}
