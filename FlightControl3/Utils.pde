// Misc Utils

// Smooth - factor in 1/16th (int) = 0-16 maps to 0-100%
int smooth(int current, int previous, int factor) {
  return ((previous * (16 - factor) + (current * factor))) / 16;
}
