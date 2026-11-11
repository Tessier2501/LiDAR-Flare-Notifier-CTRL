#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ptgl_parser.h"

#define BIN_BUF_LEN (1U << 20)

int main(int argc, char **argv)
{
  const char *bin_path = NULL;
  const char *expect_path = NULL;
  unsigned long fps = 100UL;
  int show_frames = 0;
  static uint8_t data[BIN_BUF_LEN];
  FILE *file;
  FILE *expect = NULL;
  size_t size;
  ptgl_parser_t parser;
  ptgl_frame_t frame;
  unsigned long index = 0UL;
  unsigned long valid = 0UL;
  unsigned long invalid = 0UL;
  unsigned long mismatches = 0UL;
  unsigned long first_t = 0UL;
  unsigned long first_d = 0UL;
  unsigned long last_t = 0UL;
  unsigned long last_d = 0UL;
  unsigned long range_min = 0UL;
  unsigned long range_max = 0UL;
  int have_first = 0;
  int have_range = 0;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
      fps = strtoul(argv[++i], NULL, 10);
    } else if (strcmp(argv[i], "--frames") == 0) {
      show_frames = 1;
    } else if (strcmp(argv[i], "--expect") == 0 && i + 1 < argc) {
      expect_path = argv[++i];
    } else if (bin_path == NULL) {
      bin_path = argv[i];
    } else {
      fprintf(stderr, "error: unexpected argument %s\n", argv[i]);
      return 2;
    }
  }

  if (bin_path == NULL || fps == 0UL) {
    fprintf(stderr, "usage: %s <bin> [--fps N] [--frames] [--expect PATH]\n", argv[0]);
    return 2;
  }

  file = fopen(bin_path, "rb");
  if (file == NULL) {
    perror(bin_path);
    return 2;
  }
  size = fread(data, 1U, sizeof(data), file);
  if (ferror(file)) {
    perror(bin_path);
    fclose(file);
    return 2;
  }
  if (size == sizeof(data)) {
    int extra = fgetc(file);
    if (extra != EOF) {
      fprintf(stderr, "error: input exceeds %zu bytes\n", sizeof(data));
      fclose(file);
      return 2;
    }
    if (ferror(file)) {
      perror(bin_path);
      fclose(file);
      return 2;
    }
  }
  fclose(file);
  if (size == 0U) {
    fprintf(stderr, "error: empty input\n");
    return 2;
  }

  if (expect_path != NULL) {
    expect = fopen(expect_path, "r");
    if (expect == NULL) {
      perror(expect_path);
      return 2;
    }
  }

  ptgl_parser_init(&parser);
  for (size_t i = 0U; i < size; i++) {
    ptgl_parser_feed(&parser, &data[i], 1U, 0U);
    while (ptgl_parser_pop(&parser, &frame, NULL)) {
      unsigned long t_ms = (index * 1000UL) / fps;
      unsigned long distance = (unsigned long)frame.distance_mm;

      if (frame.sta == 1U) {
        valid++;
        if ((have_range == 0) || (distance < range_min)) {
          range_min = distance;
        }
        if ((have_range == 0) || (distance > range_max)) {
          range_max = distance;
        }
        have_range = 1;
      } else {
        invalid++;
      }

      if (have_first == 0) {
        first_t = t_ms;
        first_d = distance;
        have_first = 1;
      }
      last_t = t_ms;
      last_d = distance;

      if (show_frames != 0) {
        printf("%lu %lu %lu %u\n", index, t_ms, distance, (unsigned)frame.sta);
      }

      if (expect != NULL) {
        unsigned long expected_distance;
        unsigned expected_sta;

        if (fscanf(expect, "%lu %u", &expected_distance, &expected_sta) != 2) {
          fprintf(stderr, "error: expectation exhausted at frame %lu\n", index);
          mismatches++;
        } else if ((expected_distance != distance) || (expected_sta != (unsigned)frame.sta)) {
          mismatches++;
          printf("mismatch %lu expected=%lu/%u got=%lu/%u\n", index,
                 expected_distance, expected_sta, distance, (unsigned)frame.sta);
        }
      }

      index++;
    }
  }

  printf("frames=%lu valid=%lu invalid=%lu checksum_err=%u func_err=%u\n",
         index, valid, invalid, (unsigned)parser.checksum_err, (unsigned)parser.func_err);
  if (have_first != 0) {
    printf("first t=%lums dist=%lumm\n", first_t, first_d);
    printf("last t=%lums dist=%lumm\n", last_t, last_d);
  }
  if (have_range != 0) {
    printf("range_min=%lu range_max=%lu\n", range_min, range_max);
  }
  if (expect != NULL) {
    unsigned long expected_distance;
    unsigned expected_sta;

    if (fscanf(expect, "%lu %u", &expected_distance, &expected_sta) == 2) {
      fprintf(stderr, "error: expectation has extra lines\n");
      mismatches++;
    }
    fclose(expect);
  }
  printf("mismatches=%lu\n", mismatches);

  return mismatches == 0UL ? 0 : 1;
}
