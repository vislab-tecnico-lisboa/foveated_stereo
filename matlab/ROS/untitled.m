x = randn(400,1);
y1 = nominal(randi(2,400,1),{'Yes','No'});
y2 = nominal(randi(3,400,1),{'X','Y','Z'});
y3 = nominal(randi(2,400,1),{'Hello','Goodbye'});
y = [y1,y2,y3];
hierarchicalBoxplot(x,y)

load carsmall
origin = cellstr(Origin);
idx = ismember(origin,{'Germany','Japan','USA'});
hierarchicalBoxplot(MPG(idx),{Model_Year(idx),origin(idx)})
