import java.math.*;
import java.io.*;

//import org.apache.commons.math.ode.*;
//import org.apache.commons.math.ode.nonstiff.*;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
//import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
//import org.orekit.bodies.CelestialBody;
//import org.orekit.bodies.BodyShape;
//import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.FramesFactory;
//import org.orekit.frames.SpacecraftFrame;
//import org.orekit.frames.Transform;
//import org.orekit.orbits.CircularOrbit;
//import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
//import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
//import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.*;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.TimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.time.TimeScale;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
//import org.orekit.forces.SphericalSpacecraft;
//import org.orekit.forces.radiation.*;
import org.orekit.data.DirectoryCrawler;
import org.orekit.data.DataContext;

public class TrackPV {
    static final int MODE_PV = 0;
    static final int MODE_AZALT = 1;
    static final int MODE_EQ = 2;

    static double lat, lon, alt;

    static void p2azalt(Vector3D p, double[] th) {
	/* Az convention: N=+y axis=0deg, E=+x axis=90deg */
	th[0] /* Az */ = Math.atan2(p.getX(), p.getY());
	th[1] /* El */ = Math.atan2(p.getZ(), Math.hypot(p.getX(), p.getY()));
    }

    static void p2eq(Vector3D p, double[] th) {
	Rotation eq_rot = new Rotation(Vector3D.PLUS_I, Math.PI/2 - lat,
				       RotationConvention.VECTOR_OPERATOR);
	Vector3D r = eq_rot.applyTo(p);
	/* HA convention: S=-y axis=0deg, W=-x axis=90deg */
	th[0] /* HA */ = Math.atan2(-r.getX(), -r.getY());
	th[1] /* Dec */ = Math.atan2(r.getZ(), Math.hypot(r.getX(), r.getY()));
    }
    
    static void output(int mode, double t, Vector3D p, Vector3D v) {
	double th[] = new double[2];
	switch (mode) {
	case MODE_PV:
	    System.out.println(String.format("%.3f", t) + "\t" +
			       p.getX() + "\t" + p.getY() + "\t" + p.getZ() + "\t" +
			       v.getX() + "\t" + v.getY() + "\t" + v.getZ());
	    break;
	case MODE_AZALT:
	    p2azalt(p, th);
	    System.out.println(String.format("%.3f", t) + "\t" +
			       th[0] + "\t" + th[1]);
	    break;
	case MODE_EQ:
	    p2eq(p, th);
	    System.out.println(String.format("%.3f", t) + "\t" +
			       th[0] + "\t" + th[1]);
	    break;
	default:
	    break;
	}
    }
    
    public static void main(String[] args) {
	boolean abs_time = true;
	File orekitData = new File("/opt/orekit/orekit-data");
	DataContext.getDefault().getDataProvidersManager()
	    .addProvider(new DirectoryCrawler(orekitData));
	
	TLE sat = new TLE(args[2], args[3]);
	    //TLE(/* ISS (ZARYA) -- ARISS 2023-05-24 */
	    //	  "1 25544U 98067A   23145.32602431  .00014206  00000-0  25493-3 0  9995",
	    //    "2 25544  51.6413  81.8570 0005396  18.1967 120.0030 15.50160974398246")
	
	Frame inertialFrame = FramesFactory.getEME2000();
	Frame ITRF = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
	OneAxisEllipsoid earth = new
	    OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS, 
			     Constants.WGS84_EARTH_FLATTENING, 
			     ITRF);

	/* station coordinates */
	lat = 52.1868056 * Math.PI/180.0; /* Lange Voort molen, Oegstgeest */
	lon = 4.4730814 * Math.PI/180.0;
	alt = -3.0;
	
	GeodeticPoint station =new GeodeticPoint(lat, lon, alt);
	TopocentricFrame station_frame = new TopocentricFrame(earth, station, "NL");

	TLEPropagator propagator = TLEPropagator.selectExtrapolator(sat);

	TimeScale utc = TimeScalesFactory.getUTC();
	AbsoluteDate t0 = new AbsoluteDate(args[0], utc);
	double t_end = Double.parseDouble(args[1]);
	double dt = 1.0;
	//System.out.println("# t0=" + t0);
	for (double t = 0.0; t < t_end; t += dt) {
	    AbsoluteDate tx = t0.shiftedBy(t);
	    PVCoordinates pv = propagator.getPVCoordinates(tx, station_frame);

	    Vector3D p = pv.getPosition(), v = pv.getVelocity();
	    double t_out;
	    if (abs_time) {
		long t_ms = tx.toDate(utc).getTime();
		t_out = t_ms/1000.0;
	    }
	    else {
		t_out = t;
	    }
	    output(MODE_EQ, abs_time ? t_out : t, p, v);
	}
    }
}
