# VTR Commit Procedures #

The purpose of this document is to outline how we commit our source changes to the VTR project.


## Internal developers ##

The guiding principle in internal development is to submit your work into the repository without breaking other people's work.  When you commit, make sure that the repository compiles, that the flow runs, and that you did not clobber someone else's work.  In the event that you are responsible for "breaking the build", fix the build at top priority.

We have some guidelines in place to help catch most of these problems.  They are as follows:

Before you commit, your code MUST pass the check-in regression test.  The check-in regression test is a quick way to test if any part of the Verilog-to-routing flow is broken.  To run the check-in regression test:

  1. Go to vtr\_flow/tasks/
  1. Enter the command "../scripts/run\_vtr\_task.pl -p2 checkin\_reg/"
  1. Enter the command "../scripts/parse\_vtr\_task.pl -check\_golden checkin\_reg"
  1. You may commit if the test returns [PASS](PASS.md)

Once we get an automatic build system setup, that build system will do a much more thorough run through of the regressions and mark down which svn revision is stable.  Once this system is in place, if the build is broken by someone else and you need a stable version to work off of, revert to the the latest stable build and go from there.

Everyone who is doing development must write a regression test for each major feature that they've created.  This is important because if someone (or yourself) breaks some new feature, the regression tests will catch it.

In the event a regression test is broken, the one responsible for having the test pass is in charge of determining: a) If there is a bug in the source code, in which case the source code needs to be updated to fix the bug or b) If there is a problem with the test (perhaps the quality of the tool did in fact get better or perhaps there is a bug with the test itself), in which case the test needs to be updated to reflect the new changes.  If the golden results need to be updated and you are sure that the new golden results are better, use the command "../scripts/parse\_vtr\_task.pl -create\_golden your\_regression\_test\_name\_here"

Update regularly and commit as regularly as you can.  The longer code deviates from the trunk, the more painful it is to integrate back into the trunk.

Whatever system that we come up with will not be foolproof so be conscientious about team development.


## External Developers ##

If you are working on Odin II - Merge with the latest revision of Odin II.  Submit patch/source code to ken@unb.ca

If you are working on VPR - Merge with the latest revision of VTR.  Run all regression tests found in vtr\_flow/tasks except for timing.  Submit patch/source code to vpr@eecg.utoronto.ca