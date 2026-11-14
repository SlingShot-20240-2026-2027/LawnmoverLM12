package org.firstinspires.ftc.teamcode.Auton.Ivy;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

final class PolyFit {

    private String cachedData = null;
    private int cachedDegree = -1;
    private double[] coefficients = null;
    private double mean = 0;
    private double scale = 1;
    private double minX = Double.NaN;
    private double maxX = Double.NaN;
    private int pointCount = 0;
    private int usedDegree = 0;
    private String error = "";

    double evaluate(String data, int degree, double x) {
        refit(data, degree);
        if (coefficients == null || Double.isNaN(x)) {
            return Double.NaN;
        }
        double clamped = Math.max(minX, Math.min(maxX, x));
        double u = (clamped - mean) / scale;
        double result = 0;
        double power = 1;
        for (double c : coefficients) {
            result += c * power;
            power *= u;
        }
        return result;
    }

    String describe(String data, int degree) {
        refit(data, degree);
        if (coefficients == null) {
            return "INVALID: " + error;
        }
        return String.format(Locale.US, "%d points, degree %d, %.0f-%.0f in", pointCount, usedDegree, minX, maxX);
    }

    private void refit(String data, int degree) {
        String safeData = data == null ? "" : data;
        if (safeData.equals(cachedData) && degree == cachedDegree) {
            return;
        }
        cachedData = safeData;
        cachedDegree = degree;
        coefficients = null;
        error = "";

        List<double[]> points = new ArrayList<>();
        for (String entry : safeData.split("[,;]")) {
            String trimmed = entry.trim();
            if (trimmed.isEmpty()) continue;
            String[] parts = trimmed.split(":");
            if (parts.length != 2) {
                error = "bad entry '" + trimmed + "' (use distance:value)";
                return;
            }
            try {
                double px = Double.parseDouble(parts[0].trim());
                double py = Double.parseDouble(parts[1].trim());
                if (Double.isNaN(px) || Double.isNaN(py) || Double.isInfinite(px) || Double.isInfinite(py)) {
                    error = "bad number in '" + trimmed + "'";
                    return;
                }
                points.add(new double[]{px, py});
            } catch (NumberFormatException e) {
                error = "bad number in '" + trimmed + "'";
                return;
            }
        }
        if (points.isEmpty()) {
            error = "no data points";
            return;
        }
        Collections.sort(points, (a, b) -> Double.compare(a[0], b[0]));

        int n = points.size();
        double sum = 0;
        for (double[] p : points) sum += p[0];
        mean = sum / n;
        minX = points.get(0)[0];
        maxX = points.get(n - 1)[0];
        scale = Math.max(1e-9, (maxX - minX) / 2.0);
        if (maxX - minX < 1e-9) {
            scale = 1;
        }

        int distinct = 1;
        for (int i = 1; i < n; i++) {
            if (points.get(i)[0] - points.get(i - 1)[0] > 1e-9) distinct++;
        }
        int d = Math.max(0, Math.min(Math.min(degree, 3), distinct - 1));

        int m = d + 1;
        double[][] a = new double[m][m + 1];
        for (double[] p : points) {
            double u = (p[0] - mean) / scale;
            double[] powers = new double[2 * m];
            powers[0] = 1;
            for (int k = 1; k < 2 * m; k++) powers[k] = powers[k - 1] * u;
            for (int r = 0; r < m; r++) {
                for (int c = 0; c < m; c++) a[r][c] += powers[r + c];
                a[r][m] += powers[r] * p[1];
            }
        }
        double[] solution = solve(a, m);
        if (solution == null) {
            error = "could not fit curve";
            return;
        }
        coefficients = solution;
        pointCount = n;
        usedDegree = d;
    }

    private static double[] solve(double[][] a, int m) {
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            }
            if (Math.abs(a[pivot][col]) < 1e-12) return null;
            double[] tmp = a[col];
            a[col] = a[pivot];
            a[pivot] = tmp;
            for (int r = 0; r < m; r++) {
                if (r == col) continue;
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) a[r][c] -= f * a[col][c];
            }
        }
        double[] x = new double[m];
        for (int r = 0; r < m; r++) x[r] = a[r][m] / a[r][r];
        return x;
    }
}
