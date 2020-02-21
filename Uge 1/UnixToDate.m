function UnixToDate(timecode)
    date = datestr((timecode+3600*2)/86400 + datenum(1970,1,1)); % time is always two hours behind
    fprintf('Measurement at: %s\n', date); % print time of first measurement
end