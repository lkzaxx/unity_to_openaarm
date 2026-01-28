IDE Fix Pack 2007 - Version 4.4 (2011-08-28)
(C) 2008-2011 Andreas Hausladen
Andreas.Hausladen@gmx.de


Installation:
=============
1. Start IDEFixPackReg.exe under your user account
2. Select the IDE registry keys for which you want to install the bugfix
3. Press the "Install" button


Uninstallation:
===============
1. Start IDEFixPackReg.exe under your user account
2. Press the "Uninstall" button. This will uninstall the bugfix for all 
   (not only the selected) IDE registry keys.


Changelog:
==========
2011-08-28 (4.4):
  - Added: Directory cache for $(WINDIR)\Globalization file search on IDE start

2011-08-20 (4.3):
  - Added fix for: QC #40945: Right Alt key causes expection error in Vista
  - Added fix for: Welcome page problems with "Save Desktop"
  - Improved: .NET XMLSerializer's CreateProcess call is now cached without the need to call CreateProcess.

2011-06-20 (4.2):
  - Added: Prevent IDE deadlocks
  - Added: HelpInsight parsing is done in the background when moving the mouse over an identifier
  - Added: Much improved IDE startup performance
  - Added: Some old Debugger C-RTL functions are replaced by much faster versions
  - Removed: No search for non-existing $(WINDIR)\Globalization files on IDE start (caused problems)

2011-04-22 (4.1):
  - Added: IDE start disk I/O optimizations
  - Added: No search for non-existing $(WINDIR)\Globalization files on IDE start

2011-04-18 (4.0):
  - Added: Structure View updates in the main thread are now faster
  - Added: Another Error Insight "Cannot resolve unit" fix
  - Added: fix for: Compilation takes longer and longer when compiling multiple times
  - Added: TXmlIniFile optimization for the IDE's usage pattern / dproj files are now better formated.
  - Added fix for: QC #88038: Delphi always maximizes itself on taskbar change
  - Added fix for: QC #89148: TListView ItemData streaming error
  - Fixed: The Editor Focus fix caused the IDE's menu bar to shrink if "Minimize on start" is set
  - Removed: "Call stack with IInterface parameters are resolved much faster" is now in DelphiSpeedUp

2010-09-15 (3.5):
  - Added fix for: QC #29732: Class Completion adds published section
  - Optimization: Call stack with IInterface parameters are resolved much faster

2010-03-15 (3.0):
  - Added fix for: QC #80822: ObjectInspecor: Properties are duplicated after scrolling
  - Added fix for: QC #80776: ObjectInspector shows "EditControl" instead of the real content
  - Added fix for: QC #79776: Clicking on object Inspector rejects focus

2009-12-22:
  - Added fix for: QC #75738: Debugging extremly slow
  - Added fix for: QC #68493: Switching away and back to Delphi orphans focus on Code Editor

2009-12-05:
  - Added fix for: Vista compatible main icon resource doesn't work

2009-09-03:
  - Added fix for: 64 bit Debugger assertion
  - Added fix for: Undo destroyed editor buffer
  - Added fix for: Vista 64 IDE startup delay
  - Added: QC #74646: Buffer overflow in TCustomClientDataSet.DataConvert with ftWideString
  - Fixed: TTabSheet looked strange if used with SilverThemes

2009-03-03:
  - Fixed: The AppDeActivateZOrder patch now fixes the cause instead of the symptoms
  - Added fix for: Error Insight fails to find TObject class
  - Added fix for: Possible deadlock when Error Insight calls ProcessMessages

2009-02-18:
  - Added for for IDE may select the wrong file when performing a ctrl+click on a filename
    in the editor
  - Added faster AnsiCompareFileName replacement function which speeds up the "Install Packages..." dialog

2009-02-05:
  - Added: fix for "Cannot resolve unit name" Error Insight bug.

2009-01-25:
  - Fixed: C++Builder compilation slow down caused by the ReadWrite mode fix
  - Fixed: DBGrid ScrollBar gab wasn't painted correctly in BiDiMode <> bdLeftToRight
  - Fixed: TTabSheet could throw an access violation if no PageControl was assigned to it

2008-11-24:
  - Added fix for Show Component Caption IDE bug
  - Added fix for IDE Compiler opens all files in ReadWrite mode and blocks command line compiler

2008-07-19:
  Added: Control resize fix replaces the old editor resize bugfix
  Added: Background Parser now stops if the main thread wants to do something with the compiler.

2008-05-12:
  Added: Fix for Find dialog has problems with Umlaut chars
  Removed: CodeCompletion is fixed in RAD Studio 2007 April Update

2008-04-02:
  Added: Added fix for TCustomActionList.Notification memory overwrite
  Added: Fixed some TDBGrid and TPageControl form designer flicker
  Improved: Application window tastbar button removal (should now work for all Windows versions)

2008-02-18:
  Added: CodeCompletion patch to show inherited items

2008-02-10:
  Added Application window tastbar button removal for Vista
  Added TDBText Color bugfix
  Renamed to IDE Fix Pack

2008-02-04:
  Initial release as IDEWin64Fix
