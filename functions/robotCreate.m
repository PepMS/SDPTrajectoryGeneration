function robot = robotCreate(dLinks)
    for i = 1:length(dLinks)
        L(i) = Link('d', 0, 'a', dLinks(i), 'alpha', 0);
    end
    robot = SerialLink(L, 'name', 'Planar_Robot');
end
