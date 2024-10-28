function [intersectP, check] = LineSegmentIntersection(p1, p2, p3, p4)
    % LineSegmentIntersection checks if two line segments (p1 to p2) and (p3 to p4) intersect
    % p1, p2: endpoints of the first link
    % p3, p4: endpoints of the second link
    % Returns:
    % intersectP - the intersection point
    % check - true if the segments intersect, false otherwise
    
    % Vector representations of the two line segments
    r = p2 - p1;
    s = p4 - p3;
    
    % Calculate the determinant
    denom = cross2D(r, s);
    
    if denom == 0
        % Lines are parallel, no intersection
        check = 0;
        intersectP = [];
        return;
    end
    
    t = cross2D((p3 - p1), s) / denom;
    u = cross2D((p3 - p1), r) / denom;
    
    % Check if the intersection point lies within the bounds of both segments
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        % Intersection point
        intersectP = p1 + t * r;
        check = 1;
    else
        % No intersection
        intersectP = [];
        check = 0;
    end
end

% 2D cross product (z-component only)
function result = cross2D(v1, v2)
    result = v1(1)*v2(2) - v1(2)*v2(1);
end
