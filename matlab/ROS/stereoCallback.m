function stereoCallback(~, message)
    %exampleHelperROSChatterCallback - ROS subscriber callback function to display data
    %   from the message.
    %   exampleHelperROSChatterCallback(~,MESSAGE) returns no arguments- it simply displays
    %   message content.
    %   
    %   See also ROSPublishAndSubscribeExample
    
    %   Copyright 2014 The MathWorks, Inc.
    
    disp('Chatter Callback message data: ');
    %message.PointStep      % Length of a point in bytes
    %message.RowStep        % Length of a row in bytes
    %message.Data
end