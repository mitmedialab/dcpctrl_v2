README - mf644fix.zip

Instructions for using MF644 frequency-output fix:

The MF-644 is known to crash OS X systems on R2016a and R2016b if the Frequency Output block is used. The ZIP file mf644fix.zip includes a patch for this bug. Instructions for use are:

Please find attached a ZIP-archive with two files. The
archive is password-protected, the password is "mf644" (without quotes).

Probably the easiest way to deploy the fix is:

unzip -d <MATLAB_root> mf644.fix.zip

This will overwrite two files in your existing MATLAB installation - you will
be prompted for that and maybe it's a good idea to back up these files first.
Root privileges (sudo) may be required. The fix should work with both R2016a
and the newest R2016b.