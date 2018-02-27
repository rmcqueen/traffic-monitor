#include "AppConfig.hpp"

int main(int argc, char *argv[]) {
  AppConfig main;
  Tracker tracker;
  main.setup(tracker);
  return 0;
}