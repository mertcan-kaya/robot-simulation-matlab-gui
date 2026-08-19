% Script to run all unit tests in the tests/ directory
addpath(pwd);

disp('========================================');
disp('   Running RobotSimApp Test Suite       ');
disp('========================================');

import matlab.unittest.TestSuite;
import matlab.unittest.TestRunner;
import matlab.unittest.plugins.DiagnosticsRecordingPlugin;

% Create suite from the tests folder
suite = TestSuite.fromFolder('tests');

% Create runner and run tests
runner = TestRunner.withTextOutput;
runner.addPlugin(DiagnosticsRecordingPlugin);
result = runner.run(suite);

disp('========================================');
hasFailures = any([result.Failed]) || any([result.Errored]);
if ~hasFailures
    disp('   ALL TESTS PASSED SUCCESSFULLY!       ');
else
    disp('   SOME TESTS FAILED!                   ');
    assert(~hasFailures, 'One or more unit tests failed or errored.');
end
disp('========================================');

