package org.team100.lib.util;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;

import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.WindowConstants;

import org.jfree.chart3d.Chart3D;
import org.jfree.chart3d.Chart3DFactory;
import org.jfree.chart3d.Chart3DPanel;
import org.jfree.chart3d.data.xyz.XYZDataset;
import org.jfree.chart3d.data.xyz.XYZSeries;
import org.jfree.chart3d.data.xyz.XYZSeriesCollection;
import org.jfree.chart3d.graphics3d.Dimension3D;
import org.jfree.chart3d.graphics3d.ViewPoint3D;
import org.jfree.chart3d.graphics3d.swing.DisplayPanel3D;
import org.jfree.chart3d.plot.XYZPlot;
import org.jfree.chart3d.renderer.xyz.LineXYZRenderer;
import org.junit.jupiter.api.Test;

/** example 3d chart */
public class Chart3dTest {
    private static final boolean SHOW = false;

    private static final Color RED = new Color(255, 0, 0);

    @Test
    void test0() {
        if (!SHOW)
            return;
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));

        XYZDataset<String> dataset = data();
        Chart3D chart = createChart(dataset);
        Chart3DPanel chartPanel = new Chart3DPanel(chart);

        Dimension size = new Dimension(892, 640);
        frame.setPreferredSize(size);
        chartPanel.zoomToFit(size);

        frame.add(chartPanel);
        frame.add(new DisplayPanel3D(chartPanel));

        //
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    }

    private static Chart3D createChart(XYZDataset<String> dataset) {
        Chart3D chart = Chart3DFactory.createXYZLineChart(null,
                null, dataset, "X", "Y", "Z");
        chart.setLegendBuilder(null);
        XYZPlot plot = (XYZPlot) chart.getPlot();
        // default appears to be left-handed, so invert one axis
        plot.getYAxis().setInverted(true);
        plot.setDimensions(new Dimension3D(1.0, 1.0, 1.0));
        LineXYZRenderer renderer = (LineXYZRenderer) plot.getRenderer();
        renderer.setColors(RED);

        double theta = -0.5;
        double phi = -1.3;
        double roll = 0;
        chart.setViewPoint(new ViewPoint3D(theta, phi, 10, roll));
        return chart;
    }

    private static XYZDataset<String> data() {
        double segment_size = 0.1;
        XYZSeriesCollection<String> dataset = new XYZSeriesCollection<>();
        double t = 0;
        for (int i = 0; i < 120; ++i) {
            double x = Math.cos(t);
            double y = Math.sin(t);
            double z = t;
            double dx = -Math.sin(t) * segment_size;
            double dy = Math.cos(t) * segment_size;
            double dz = segment_size;
            dataset.add(arrow(i, x, y, z, dx, dy, dz));
            t += segment_size;
        }
        return dataset;
    }

    private static XYZSeries<String> arrow(
            int j, double x0, double y0, double z0, double dx, double dy, double dz) {
        double x1 = x0 + dx;
        double y1 = y0 + dy;
        double z1 = z0 + dz;
        XYZSeries<String> s = new XYZSeries<>(String.format("%d", j));
        s.add(x0, y0, z0);
        s.add(x1, y1, z1);
        double arrowhead_size = 0.25;
        double ax = dx * arrowhead_size;
        double ay = dy * arrowhead_size;
        double az = dz * arrowhead_size;
        // permute
        s.add(
                x1 - (ax + ay / 2),
                y1 - (ay + az / 2),
                z1 - (az + ax / 2));
        return s;
    }
}
