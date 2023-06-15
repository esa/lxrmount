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
import org.orekit.frames.Transform;
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

public class ITRF2ab {
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
    
    public static void main(String[] args)
	throws IOException
    {
	File orekitData = new File("/opt/orekit/orekit-data");
	DataContext.getDefault().getDataProvidersManager()
	    .addProvider(new DirectoryCrawler(orekitData));
	
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

	TimeScale utc = TimeScalesFactory.getUTC();
	BufferedReader inp = new BufferedReader(new InputStreamReader(System.in));
	for (;;) {
	    String ln = inp.readLine();
	    if (ln == null)
		break;
	    String fld[] = ln.split(" +");
	    if (fld.length != 7)
		continue;

	    AbsoluteDate t = new AbsoluteDate(fld[0], utc);
	    Vector3D p_inp = new Vector3D
		(Double.parseDouble(fld[1]),
		 Double.parseDouble(fld[2]),
		 Double.parseDouble(fld[3])).scalarMultiply(1e3),
		v_inp = new Vector3D
		(Double.parseDouble(fld[4]),
		 Double.parseDouble(fld[5]),
		 Double.parseDouble(fld[6])).scalarMultiply(1e3);

	    //System.out.println(t.toString() +p_inp.toString() +v_inp.toString());

	    long t_ms = t.toDate(utc).getTime();
	    double t_out = t_ms/1000.0;

	    Transform itrf2topo = ITRF.getTransformTo(station_frame, t);
	    Vector3D
		p_out = itrf2topo.transformPosition(p_inp),
		v_out = itrf2topo.transformVector(v_inp);
	    
	    output(MODE_EQ, t_out, p_out, v_out);
	    //output(MODE_PV, t_out, p_out, v_out);
	}
    }
}
