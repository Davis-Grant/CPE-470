function  [time, data] = rtpload(filename)
fid = fopen(filename);
if (fid < 0)
  error('Unable to open file %s', filename);
end

line = fgetl(fid);
fclose(fid);

% Make sure the file contains something.
if (line <0)
  error('Empty file %s', filename);
end

% Load the actual data.
raw = load(filename);

% Restructure the data.
column = 0;
while (~isempty(line))
  [token,line] = strtok(line,'%,');
  column = column+1;
  eval([token ' = raw(:,' num2str(column) ');']);
end

% Move to the correct output variables.  The first column
% is 'time', the rest are 'field.item1' 'field.item2' etc.
time = time;
data = field;
return;