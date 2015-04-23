// project-stubs.c
// 
// Contains stub versions of all of the projectX_Y functions used by Roger.
// These call exported Haskell functions, passing in the Roger instance, the
// current time, and/or a global state variable, as appropriate.
//
// All exported Haskell functions start with "hs_".

#include <HsFFI.h>
#include "roger.h"
#include "control.h"
#include "HaskellExports_stub.h"

// -----------------------------------------------------------------------------
// Project 1

void control_roger(Robot* roger, double time) {
  void update_setpoints();
  update_setpoints(roger); // check_GUI_inputs(roger)
  hs_control_roger(roger, time);
}

void project1_reset(Robot* roger) {}
void project1_enter_params() {}
void project1_visualize() {}

// -----------------------------------------------------------------------------
// Project 2

static HsStablePtr project2_state = 0;
static HsStablePtr project2_get_state() {
  if (project2_state == 0) project2_state = hs_project2_init_state();
  return project2_state;
}

void project2_control(Robot* roger, double time) {
  project2_state = hs_project2_control(roger, project2_get_state(), time);
}
void project2_reset(Robot* roger) {
  project2_state = hs_project2_init_state();
}
void project2_enter_params() {
  project2_state = hs_project2_enter_params(project2_get_state());
}
void project2_visualize() {}
int inv_arm_kinematics(Robot* roger, int limb, double x, double y) {
  int ret;
  project2_state = hs_inv_arm_kinematics(roger, project2_get_state(), limb, x, y, &ret);
  return ret;
}

// -----------------------------------------------------------------------------
// Project 3

static HsStablePtr project3_state = 0;
static HsStablePtr project3_get_state() {
  if (project3_state == 0) project3_state = hs_project3_init_state();
  return project3_state;
}

void project3_control(Robot* roger, double time) {
  project3_state = hs_project3_control(roger, project3_get_state(), time);
}
void project3_reset(Robot* roger) {
  project3_state = hs_project3_init_state();
}
void project3_enter_params() {
  project3_state = hs_project3_enter_params(project3_get_state());
}
void project3_visualize() {}

// -----------------------------------------------------------------------------
// Project 4

static HsStablePtr project4_state = 0;
static HsStablePtr project4_get_state() {
  if (project4_state == 0) project4_state = hs_project4_init_state();
  return project4_state;
}

void project4_control(Robot* roger, double time) {
  project4_state = hs_project4_control(roger, project4_get_state(), time);
}
void project4_reset(Robot* roger) {
  project4_state = hs_project4_init_state();
}
void project4_enter_params() {
  project4_state = hs_project4_enter_params(project4_get_state());
}
void project4_visualize() {}

// -----------------------------------------------------------------------------
// Project 5

static HsStablePtr project5_state = 0;
static HsStablePtr project5_get_state() {
  if (project5_state == 0) project5_state = hs_project5_init_state();
  return project5_state;
}

void project5_control(Robot* roger, double time) {
  project5_state = hs_project5_control(roger, project5_get_state(), time);
}
void project5_reset(Robot* roger) {
  project5_state = hs_project5_init_state();
}
void project5_enter_params() {
  project5_state = hs_project5_enter_params(project5_get_state());
}
void project5_visualize() {
  //Observation obs;
  //void draw_observation();
  //hs_project5_get_observation(project5_get_state(), &obs);
  //draw_observation(obs);
}

// -----------------------------------------------------------------------------
// Project 6

static HsStablePtr project6_state = 0;
static HsStablePtr project6_get_state() {
  if (project6_state == 0) project6_state = hs_project6_init_state();
  return project6_state;
}

void project6_control(Robot* roger, double time) {
  project6_state = hs_project6_control(roger, project6_get_state(), time);
}
void project6_reset(Robot* roger) {
  project6_state = hs_project6_init_state();
}
void project6_enter_params() {
  project6_state = hs_project6_enter_params(project6_get_state());
}
void project6_visualize() {}

// -----------------------------------------------------------------------------
// Project 7

static HsStablePtr project7_state = 0;
static HsStablePtr project7_get_state() {
  if (project7_state == 0) project7_state = hs_project7_init_state();
  return project7_state;
}

void project7_control(Robot* roger, double time) {
  project7_state = hs_project7_control(roger, project7_get_state(), time);
}
void project7_reset(Robot* roger) {
  project7_state = hs_project7_init_state();
}
void project7_enter_params() {
  project7_state = hs_project7_enter_params(project7_get_state());
}
void project7_visualize() {}

// -----------------------------------------------------------------------------
// Project 8

static HsStablePtr project8_state = 0;
static HsStablePtr project8_get_state() {
  if (project8_state == 0) project8_state = hs_project8_init_state();
  return project8_state;
}

void project8_control(Robot* roger, double time) {
  project8_state = hs_project8_control(roger, project8_get_state(), time);
}
void project8_reset(Robot* roger) {
  project8_state = hs_project8_init_state();
}
void project8_enter_params() {
  project8_state = hs_project8_enter_params(project8_get_state());
}
void project8_visualize() {}

void sor() {}

double compute_gradient(double x, double y, Robot* roger, double grad[2]) {
  return 0.0;
}

// -----------------------------------------------------------------------------
// Project 9

static HsStablePtr project9_state = 0;
static HsStablePtr project9_get_state() {
  if (project9_state == 0) project9_state = hs_project9_init_state();
  return project9_state;
}

void project9_control(Robot* roger, double time) {
  project9_state = hs_project9_control(roger, project9_get_state(), time);
}
void project9_reset(Robot* roger) {
  project9_state = hs_project9_init_state();
}
void project9_enter_params() {
  project9_state = hs_project9_enter_params(project9_get_state());
}
void project9_visualize() {}

