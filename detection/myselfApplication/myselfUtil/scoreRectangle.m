function rectScore = scoreRectangle(curI, curD, curN, curMask, curDMask, featsz, mask_rsz_thresh, featMeans, featStds, trainModes, w1, w2, w_class, firstRow, firstCol, height, width)
    % Have a valid candidate rectangle
    % Extract features for the current rectangle into the
    % format the DBN expects
    [curFeat, curFeatMask] = featForRect(curI,curD,curN,curDMask,firstRow,firstCol,height,width,featsz,mask_rsz_thresh);
    curFeat = simpleWhiten(curFeat,featMeans,featStds);
    curFeat = scaleFeatForMask(curFeat, curFeatMask, trainModes);

    % Run the features through the DBN and get a score.
    % Might be more efficient to collect features for a
    % group of rectangles and run them all at once
    w1Probs = 1./(1+exp(-[curFeat 1]*w1));
    w2Probs = 1./(1+exp(-[w1Probs 1]*w2));
    rectScore = [w2Probs 1]* w_class;
end